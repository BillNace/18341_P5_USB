`default_nettype none
`include "USBPkg.pkg"

// USB DP DM interface
interface USBWires;
  tri0 DP;
  tri0 DM;
endinterface

//////
////// USB Serial Interface Engine 18-341
////// (created S'17, Daniel Stiffler)
//////

////
//// The USB SIE Testbench
////
////   clock         (input)  - The clock
////   reset_n       (input)  - Asynchronous reset
////   wires         (iface)  - USB {DP DM} tri-state wires, pulled down to 0
////   debug_pkt_ext ()       - Exposed USB packet struct from the bus analyzer
////                            that reflects captured traffic in real-time.
////                            See field definitons in "usbPkg.pkg".
////
//// Testbench Usage
////   1. Compile your code using the Makefile supplied "make {full/clean}"
////   2. Run the testbench with one or more of the following runtime arguments
////      ./simv {-gui} {+{plusargs ... }} {+vcs+finish+{d}} {+VERBOSE={1,2,3}}
////      e.g. ./simv -gui +SIMPLE +EDGE +vcs+finish+100000 +VERBOSE=3
////
////     - +PRELAB:  prelab checkoff; device accepts a valid OUT packet with
////                 ENDP=`DATA_ENDP and ADDR=`DEVICE_ADDR
////     - +SIMPLE:  TB directs host to write then read one random address
////     - +EDGE:    TB directs host to write then read carefully chosen
////                 addresses and data to cause CRC and bitstuffing edge-
////                 cases
////     - +STRESS:  TB directs host to write then read 100 random addresses
////     - +CORRUPT: TB directs host to read 10 random addresses, but the
////                 device will send between 1 and 6 corrupt versions of the
////                 DATA0 packet each time; YOU WILL SEE AN ASSERTION FAILURE
////     - +TIMEOUT: TB directs host to read 10 random addresses, but the device
////                 will timeout between 1 and 6 times
////     - +NAK:     TB directs host to write 10 random addresses, but the
////                 device will send 1 to 3 bogus NAKs during each OUT
////                 transaction; YOU WILL SEE AN ASSERTION FAILURE
////     - +ABORT:   TB directs host to write random data twice, but the device
////                 will error 8 times and throw out both transactions; YOU
////                 WILL SEE AN ASSERTION FAILURE
////
////   !NOTE AGAIN!: concurrent assertions are statements of truth, so YOU WILL
////                 see some very specific failures in +CORRUPT +NAK and +ABORT
////
module USBTB;
  logic clock, reset_n;
  default clocking cb_main @(posedge clock); endclocking

  logic host_success;
  logic [63:0] data_to_host, data_from_host;
  logic [15:0] mempage;

  integer write_errors, read_errors, fake_errors;

  // CRC and bit-stuffing edge cases
  typedef enum logic [63:0] {
      A=64'ha7_4f_00_00_00_00_00_00, B=64'h00_00_00_00_8e_f1_4f_37,
      C=64'hfb_65_67_6d_11_e1_c5_b9
  } crc_edge_t;
  crc_edge_t crc_edge;

  debug_pkt_t debug_pkt_ext;
  USBWires wires();

  USBDevice #(.DEVICE_ADDR(`DEVICE_ADDR), .ADDR_ENDP(`ADDR_ENDP),
              .DATA_ENDP(`DATA_ENDP))
      device_inst (.*);

  USBHost host_inst (.*);

  integer debug_level = 0;
  initial $value$plusargs("VERBOSE=%d", debug_level);

  // Conduct a system reset
  task doReset;
    $srandom(18341);

    reset_n = 1'b1;
    reset_n <= 1'b0;

    #1 reset_n <= 1'b1;
  endtask : doReset

  initial begin
    clock = 1'b0;
    forever #5 clock = ~clock;
  end

  // Unconditional timeout so that students do not have to worry about hanging
  initial begin
    ##10000000 ;
    $display("%m @%0t: Testbench issued timeout", $time);
    $finish;
  end

  initial begin
    if ($test$plusargs("PRELAB")) begin
      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Started> Prelab\n",
                "-----------------------------------------------------------",
                "\n"});

      doReset;
      ##1 ;

      $display("%m @%0t: TB->Host requesting OUT to addr=%0d endp=%0d",
               $time, `DEVICE_ADDR, `ADDR_ENDP);

      device_inst.setPrelab();

      fork
        begin
          host_inst.prelabRequest();
        end

        begin
          @(posedge device_inst.device_done_tx);

          $display({"%m @%0t: Device saw correct prelab packet. Prelab ",
                    "successful"},
                   $time);
        end
      join_any
      ##2 ; // Have to check a cycle later

      assert(device_inst.device_idle) else begin
        $error("Device is still attempting to finish the transaction");
        $finish;
      end
    end

    if ($test$plusargs("SIMPLE")) begin
      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Started> Simple Read/Write Test\n",
                "-----------------------------------------------------------",
                "\n"});

      mempage = $urandom;
      data_to_host = {$urandom, $urandom};
      doReset;
      ##1 ;

      $display("%m @%0t: TB->Host writing mempage %x and data %x",
               $time, mempage, data_to_host);

      host_inst.writeData(mempage, data_to_host, host_success);

      if (host_success && device_inst.device_idle
          && device_inst.memory[mempage] == data_to_host) begin

        $display("%m @%0t: Host and device reported successful write", $time);
      end else begin
        assert(host_success) else begin
          $error("Host reported unsuccessful write");
          $finish;
        end

        assert(device_inst.device_idle) else begin
          $error("Device is still attempting to finish the transaction");
          $finish;
        end

        assert(device_inst.memory[mempage] == data_to_host) else
          $error("Device wrote the wrong value to memory[%x]=%x",
                 mempage, device_inst.memory[mempage]);
      end

      $display("%m @%0t: TB->Host reading mempage %x", $time, mempage);
      host_inst.readData(mempage, data_from_host, host_success);

      if (host_success && device_inst.device_idle
          && data_from_host == data_to_host) begin

        $display("%m @%0t: Host and device reported successful read", $time);
      end else begin
        assert(host_success) else begin
          $error("Host reported unsuccessful read");
          $finish;
        end

        assert(device_inst.device_idle) else begin
          $error("Device is still attempting to finish the transaction");
          $finish;
        end

        assert(data_from_host == data_to_host) else
          $error("Host read the wrong value from memory[%x]=%x (saw %x)",
                 mempage, data_to_host, data_from_host);
      end

      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Finished> Simple Read/Write Test\n",
                "-----------------------------------------------------------",
                "\n"});
    end

    if ($test$plusargs("EDGE")) begin
      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Started> Edge Case Read/Write Test\n",
                "-----------------------------------------------------------",
                "\n"});

      doReset;
      ##1 ;

      crc_edge = crc_edge.first;
      do begin
        host_inst.writeData(16'h00_00, crc_edge, host_success);

        if (host_success && device_inst.device_idle
            && device_inst.memory[16'h00_00] == crc_edge) begin

            $display("%m @%0t: Host and device reported successful write",
                     $time);

        end else begin
          assert(host_success) else begin
            $error("Host reported unsuccessful write");
            $finish;
          end

          assert(device_inst.device_idle) else begin
            $error("Device is still attempting to finish the transaction");
            $finish;
          end

          assert(device_inst.memory[mempage] == crc_edge) else
            $error("Device wrote the wrong value to memory[%x]=%x",
                   mempage, device_inst.memory[mempage]);
        end

        crc_edge = crc_edge.next;
      end while (crc_edge != crc_edge.first);

      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Finished> Edge Case Read/Write Test\n",
                "-----------------------------------------------------------",
                "\n"});
    end

    if ($test$plusargs("STRESS")) begin
      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Started> Stress Read/Write Test\n",
                "-----------------------------------------------------------",
                "\n"});

      write_errors = 0;
      read_errors = 0;

      doReset;
      ##1 ;

      $srandom(18341);
      repeat(100) begin
        mempage = $urandom;
        data_to_host = {mempage, 16'h00_00, ~mempage, 16'h11_11};

        host_inst.writeData(mempage, data_to_host, host_success);
        assert(host_success & device_inst.device_idle) else begin
          $error({"Write to memory[%x]=%x was unsuccessful due to protocol ",
                  "error"},
                 mempage, data_to_host);

          $finish;
        end

        if (device_inst.memory[mempage] != data_to_host) begin
          $error("Write to memory[%x]=%x was incorrect (saw %x)",
                 mempage, data_to_host, device_inst.memory[mempage]);

          write_errors += 1;
        end
      end

      $srandom(18341);
      repeat(100) begin
        mempage = $urandom;
        data_to_host = {mempage, 16'h00_00, ~mempage, 16'h11_11};

        host_inst.readData(mempage, data_from_host, host_success);
        assert(host_success & device_inst.device_idle) else begin
          $error({"Read from memory[%]=%x was unsuccessful due to protocol ",
                  "error"},
                 mempage, data_to_host);

          $finish;
        end

        if (data_from_host != data_to_host) begin
          $error("Read from memory[%x]=%x was incorrect (saw %x)",
                 mempage, data_to_host, data_from_host);

          read_errors += 1;
        end
      end

      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Finished> Stress Read/Write Test \n",
                "   Write Errors: %0d/100\n",
                "   Read Errors: %0d/100\n",
                "-----------------------------------------------------------",
                "\n"},
               write_errors, read_errors);
    end

    if ($test$plusargs("CORRUPT")) begin
      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Started> Corrupt Read/Write Test\n",
                "   The `device_inst.chk_crc16` assertion will fail on DATA0 ",
                "   packets sent by the USBDevice... keep this in mind.\n",
                "-----------------------------------------------------------",
                "\n"});

      doReset;
      ##1 ;

      repeat (10) begin
        mempage = $urandom;
        fake_errors = $urandom_range(1, 6);
        device_inst.setFakeCorrupt(fake_errors);
        $display("%m @%0t: TB->Host reading with %0d fake corrupt errors",
                 $time, fake_errors);

        fork
          begin
            host_inst.readData(mempage, data_from_host, host_success);
          end

          begin
            @(posedge device_inst.device_done_tx);

            assert(~device_inst.device_null_tx) else begin
              $error("Device aborted the transaction early");
              $finish;
            end
          end
        join

        if (host_success && device_inst.device_idle
            && device_inst.memory[mempage] == data_from_host) begin

          $display("%m @%0t: Host and device reported successful read",
                   $time);
        end else begin
          if (~device_inst.device_idle) begin
            $display({"%m @%0t: Device is still attempting to finish the ",
                      "transaction"},
                     $time);

            $finish;
          end else if (device_inst.memory[mempage] != data_from_host) begin
            $display("%m @%0t: Device read the wrong value from memory[%x]",
                     $time, mempage);
            $finish;
          end else begin
            $display("%m @%0t: Host reported unsuccessful read", $time);
            $finish;
          end
        end
      end

      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Finished> Corrupt Read/Write Test\n",
                "-----------------------------------------------------------",
                "\n"});
    end

    if ($test$plusargs("TIMEOUT")) begin
      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Started> Timeout Read/Write Test\n",
                "-----------------------------------------------------------",
                "\n"});

      doReset;
      ##1 ;

      repeat (10) begin
        mempage = $urandom;
        fake_errors = $urandom_range(1, 6);
        device_inst.setFakeTimeout(fake_errors);
        $display("%m @%0t: TB->Host reading with %0d fake timeout errors",
                 $time, fake_errors);

        fork
          begin
            host_inst.readData(mempage, data_from_host, host_success);
          end

          begin
            @(posedge device_inst.device_done_tx);

            if (device_inst.device_null_tx) begin
              $display("%m @%0t: Device aborted the transaction early", $time);
              $finish;
            end
          end
        join

        if (host_success && device_inst.device_idle
            && device_inst.memory[mempage] == data_from_host) begin
          $display("%m @%0t: Host and device reported successful read",
                   $time);
        end else begin
          assert(host_success) else begin
            $error("Host reported unsuccessful read");
            $finish;
          end

          assert(device_inst.device_idle) else begin
            $error("Device is still attempting to finish the transaction");
            $finish;
          end

          assert(device_inst.memory[mempage] == data_from_host) else begin
            $error("Host read the wrong value from memory[%x]=%x (saw %x)",
                   mempage, data_to_host, data_from_host);
            $finish;
          end
        end
      end

      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Finished> Timeout Read/Write Test\n",
                "-----------------------------------------------------------",
                "\n"});
    end

    if ($test$plusargs("NAK")) begin
      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Started> NAK Spam Test\n",
                "   The `device_inst.chk_protocol` assertion will fail on NAK ",
                "   packets sent by the USBDevice... keep this in mind.\n",
                "-----------------------------------------------------------",
                "\n"});

      doReset;
      ##1 ;

      repeat (10) begin
        mempage = $urandom;
        data_to_host = {$urandom, $urandom};
        fake_errors = $urandom_range(1, 3);
        device_inst.setFakeNAKAddr(fake_errors);
        device_inst.setFakeNAKData(fake_errors);
        $display("%m @%0t: TB->Host reading with %0dx2 fake NAK errors",
                 $time, fake_errors);

        fork
          begin
            host_inst.writeData(mempage, data_to_host, host_success);
          end

          begin
            @(posedge device_inst.device_done_tx);

            if (device_inst.device_null_tx) begin
              $display("%m @%0t: Device aborted the transaction early", $time);
              $finish;
            end
          end
        join

        if (host_success && device_inst.device_idle
            && device_inst.memory[mempage] == data_to_host) begin

          $display("%m @%0t: Host and device reported successful write",
                   $time);
        end else begin
          assert(host_success) else begin
            $error("Host reported unsuccessful write");
            $finish;
          end

          assert(device_inst.device_idle) else begin
            $error("Device is still attempting to finish the transaction");
            $finish;
          end

          assert(device_inst.memory[mempage] == data_to_host) else begin
            $error("Device wrote the wrong value to memory[%x]=%x",
                   mempage, data_to_host);
            $finish;
          end
        end
      end

      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Finished> NAK Spam Test\n",
                "-----------------------------------------------------------",
                "\n"});
    end

    if ($test$plusargs("ABORT")) begin
      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Started> Transaction Abort Test\n",
                "   The `device_inst.chk_protocol` assertion will fail on ",
                "   missing packets sent by the USBDevice... keep this in ",
                "   mind.\n",
                "-----------------------------------------------------------",
                "\n"});

      doReset;
      ##1 ;

      device_inst.setFakeNAKAddr(8);
      device_inst.setFakeNAKData(0);
      $display("%m @%0t: TB->Host attempting to abort transaction in ADDR",
               $time);

      fork
        begin
          host_inst.writeData('0, '0, host_success);
        end

        begin
          @(posedge device_inst.device_done_tx);

          assert(device_inst.device_null_tx) begin
            $display("%m @%0t: Device successfully aborted the transaction",
                     $time);
          end else begin
            $error("Device unsuccessfully aborted the transaction");
            $finish;
          end
        end
      join

      if (~host_success && device_inst.device_idle) begin
        $display("%m @%0t: Host and device aborted the transaction in ADDR",
                 $time);
      end else begin
        assert(~host_success) else begin
          $error("Host mistakenly reported successful write");
          $finish;
        end

        assert(device_inst.device_idle) else begin
          $error("Device is still attempting to finish the transaction");
          $finish;
        end
      end

      device_inst.setFakeNAKAddr(0);
      device_inst.setFakeNAKData(8);
      $display("%m @%0t: TB->Host attempting to abort transaction in DATA",
               $time);

      fork
        begin
          host_inst.writeData('0, '0, host_success);
        end

        begin
          @(posedge device_inst.device_done_tx);

          assert(device_inst.device_null_tx) begin
            $display("%m @%0t: Device successfully aborted the transaction",
                     $time);
          end else begin
            $error("Device unsuccessfully aborted the transaction");
            $finish;
          end
        end
      join

      if (~host_success && device_inst.device_idle) begin
        $display("%m @%0t: Host and device aborted the transaction in DATA",
                 $time);
      end else begin
        assert(~host_success) else begin
          $error("Host mistakenly reported successful write");
          $finish;
        end

        assert(device_inst.device_idle) else begin
          $error("Device is still attempting to finish the transaction");
          $finish;
        end
      end

      $display({"\n",
                "-----------------------------------------------------------\n",
                " <Finished> Transaction Abort Test\n",
                "-----------------------------------------------------------",
                "\n"});
    end

    $finish;
  end
endmodule : USBTB
