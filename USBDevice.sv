`default_nettype none
`include "USB.svh"
`include "USBPkg.pkg"

//////
////// USB Serial Interface Engine 18-341
////// Device module
//////
module USBDevice #(parameter DEVICE_ADDR=`DEVICE_ADDR, DATA_ENDP=`DATA_ENDP,
                             ADDR_ENDP=`ADDR_ENDP) (
    USBWires wires,
    input logic clock, reset_n,
    output debug_pkt_t debug_pkt_ext);

  `protect
  /*--------------------------------------------------------------------------
   * Testbench Fault Injection
   *   These tasks behave strangely since they mix blocking assignment with
   *   clock events, but this is necessary
   *--------------------------------------------------------------------------*/
  logic ld_do_prelab_TB, do_prelab, dec_do_prelab;

  logic [3:0] fake_nak_addr_TB, fake_nak_data_TB, fake_corrupt_TB,
              fake_timeout_TB;
  logic ld_fake_nak_addr_TB, ld_fake_nak_data_TB, ld_fake_corrupt_TB,
        ld_fake_timeout_TB;

  logic [3:0] fake_nak_addr, fake_nak_data, fake_corrupt, fake_timeout;
  logic dec_fake_nak_addr, dec_fake_nak_data, dec_fake_corrupt,
        dec_fake_timeout;

  /*
   * Terminate protocol on successful receipt of OUT packet to begin OUT ADDR
   * transaction
   */
  task setPrelab();
    ld_do_prelab_TB <= 1'b1;
    @(posedge clock);
    ld_do_prelab_TB <= 1'b0;
  endtask : setPrelab

  // Send a fake NAK on the host's DATA0 during the OUT ADDR transaction
  task setFakeNAKAddr(input logic [3:0] count);
    fake_nak_addr_TB <= count;
    ld_fake_nak_addr_TB <= 1'b1;
    @(posedge clock);
    ld_fake_nak_addr_TB <= 1'b0;
  endtask : setFakeNAKAddr

  // Send a fake NAK on the host's DATA0 during the OUT DATA transaction
  task setFakeNAKData(input logic [3:0] count);
    fake_nak_data_TB <= count;
    ld_fake_nak_data_TB <= 1'b1;
    @(posedge clock);
    ld_fake_nak_data_TB <= 1'b0;
  endtask : setFakeNAKData

  // Send a fake corrupt DATA0 packet during the IN DATA trasnaction
  task setFakeCorrupt(input logic [3:0] count);
    fake_corrupt_TB <= count;
    ld_fake_corrupt_TB <= 1'b1;
    @(posedge clock);
    ld_fake_corrupt_TB <= 1'b0;
  endtask : setFakeCorrupt

  // Send a fake timeout during the IN DATA trasnaction
  task setFakeTimeout(input logic [3:0] count);
    fake_timeout_TB <= count;
    ld_fake_timeout_TB <= 1'b1;
    @(posedge clock);
    ld_fake_timeout_TB <= 1'b0;
  endtask : setFakeTimeout

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      do_prelab <= 1'b0;
      fake_nak_addr <= '0;
      fake_nak_data <= '0;
      fake_corrupt <= '0;
      fake_timeout <= '0;
    end else begin
      // Testbench manipulation from tasks
      if (ld_do_prelab_TB) do_prelab <= 1'b1;
      if (ld_fake_nak_addr_TB) fake_nak_addr <= fake_nak_addr_TB;
      if (ld_fake_nak_data_TB) fake_nak_data <= fake_nak_data_TB;
      if (ld_fake_corrupt_TB) fake_corrupt <= fake_corrupt_TB;
      if (ld_fake_timeout_TB) fake_timeout <= fake_timeout_TB;

      // Datapath manipulation
      if (dec_do_prelab) begin
        do_prelab <= 1'b0;
      end
      if (dec_fake_nak_addr && fake_nak_addr != 0) begin
        fake_nak_addr <= fake_nak_addr - 1;
      end
      if (dec_fake_nak_data && fake_nak_data != 0) begin
        fake_nak_data <= fake_nak_data - 1;
      end
      if (dec_fake_corrupt && fake_corrupt != 0) begin
        fake_corrupt <= fake_corrupt - 1;
      end
      if (dec_fake_timeout && fake_timeout != 0) begin
        fake_timeout <= fake_timeout - 1;
      end
    end
  end

  /*--------------------------------------------------------------------------
   * USB Read/Write Protocol FSMD
   *--------------------------------------------------------------------------*/
  logic device_idle, device_done_tx, device_null_tx;

  logic ready_outbound, ready_inbound, send_corrupt, done_sending,
        done_receiving;
  pkt_t pkt_outbound, pkt_inbound;
  logic recv_error, recv_timeout;
  logic ld_addr_inbound;
  logic [15:0] addr_inbound;
  logic [63:0] payload_inbound, payload_outbound;
  logic ld_payload_inbound, write_en, read_en;
  logic [63:0] memory[`LAST_ADDR];

  logic en_failed, clr_failed;
  logic [2:0] failed;

  enum {
      PROTO_IDLE, OUT_ADDR, DATA0_ADDR, NAK_ADDR, ACK_ADDR,
      IN_READ, DATA0_READ, NAK_READ,
      OUT_WRITE, DATA0_WRITE, NAK_WRITE
  } current_state, next_state;

  _USBAnalyzer #(.DEVICE_ADDR(DEVICE_ADDR), .DATA_ENDP(DATA_ENDP),
                 .ADDR_ENDP(ADDR_ENDP))
    analyzer (.wires,
              .clock, .reset_n,
              .debug_pkt_ext);

  _USBDeviceRecv
    receiver (.wires,
              .clock, .reset_n,
              .ready(ready_inbound), .done_receiving,
              .pkt_inbound,
              .recv_error, .recv_timeout);

    _USBDeviceSend
      sender (.wires,
              .clock, .reset_n,
              .ready(ready_outbound), .done_sending,
              .pkt_outbound, .send_corrupt);

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) addr_inbound <= '0;
    else if (ld_addr_inbound) addr_inbound <= pkt_inbound.payload[63:48];
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) payload_inbound <= '0;
    else if (ld_payload_inbound) payload_inbound <= pkt_inbound.payload;
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      memory <= '{default: '0};
      payload_outbound <= '0;
    end else begin
      if (write_en) memory[addr_inbound] <= payload_inbound;
      else if (read_en) payload_outbound <= memory[addr_inbound];
    end
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) failed <= '0;
    else if (clr_failed) failed <= '0;
    else if (en_failed) failed <= failed + 1;
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) current_state <= PROTO_IDLE;
    else current_state = next_state;
  end

  always_comb begin
    dec_do_prelab = 1'b0;
    dec_fake_nak_addr = 1'b0;
    dec_fake_nak_data = 1'b0;
    dec_fake_corrupt = 1'b0;
    dec_fake_timeout = 1'b0;

    device_idle = 1'b0;
    device_done_tx = 1'b0;
    device_null_tx = 1'b0;

    ready_outbound = 1'b0;
    ready_inbound = 1'b0;
    send_corrupt = 1'b0;

    pkt_outbound = '{pid: PID_X, addr: 'x, endp: 'x, payload: 'x};
    ld_payload_inbound = 1'b0;
    ld_addr_inbound = 1'b0;
    write_en = 1'b0;
    read_en = 1'b0;

    en_failed = 1'b0;
    clr_failed = 1'b0;

    next_state = current_state;

    unique case (current_state)
      PROTO_IDLE: begin
        device_idle = 1'b1;

        ready_inbound = 1'b1;

        if (done_receiving) begin
          device_idle = 1'b0;

          if (pkt_inbound.pid == PID_OUT && pkt_inbound.addr == DEVICE_ADDR
              && pkt_inbound.endp == ADDR_ENDP) begin
            if (do_prelab) begin
              dec_do_prelab = 1'b1;

              device_done_tx = 1'b1;

              next_state = PROTO_IDLE;
            end else begin
              next_state = OUT_ADDR;
            end
          end
        end
      end

      OUT_ADDR: begin
        ready_inbound = 1'b1;

        if (done_receiving) begin
          // Send legitimate NAK
          if (recv_error || recv_timeout || pkt_inbound.pid != PID_DATA0) begin
            en_failed = 1'b1;

            next_state = NAK_ADDR;
          end else if (pkt_inbound.pid == PID_DATA0) begin
            if (fake_nak_addr != 0) begin // Send fake NAKs on good packet
              en_failed = 1'b1;

              next_state = NAK_ADDR;
            end else begin // Continue with transaction
              ld_addr_inbound = 1'b1;

              clr_failed = 1'b1;

              next_state = DATA0_ADDR;
            end
          end
        end
      end

      DATA0_ADDR: begin
        ready_outbound = 1'b1;

        pkt_outbound = '{pid: PID_ACK, addr: 'x, endp: 'x, payload: 'x};

        if (done_sending) next_state = ACK_ADDR;
      end

      NAK_ADDR: begin
        ready_outbound = 1'b1;

        pkt_outbound = '{pid: PID_NAK, addr: 'x, endp: 'x, payload: 'x};

        if (done_sending) begin
          if (failed == `TX_RETRIES-1) begin // Transaction thrown out
            device_done_tx = 1'b1;
            device_null_tx = 1'b1;

            clr_failed = 1'b1;

            next_state = PROTO_IDLE;
          end else if (fake_nak_addr != 0) begin // Continue sending fake NAKs
            dec_fake_nak_addr = 1'b1;

            next_state = OUT_ADDR;
          end else begin // Continue with normal transaction
            next_state = OUT_ADDR;
          end
        end
      end

      ACK_ADDR: begin
        ready_inbound = 1'b1;

        if (done_receiving) begin
          // USB read transaction
          unique if (pkt_inbound.pid == PID_IN
                     && pkt_inbound.addr == DEVICE_ADDR
                     && pkt_inbound.endp == DATA_ENDP) begin
            read_en = 1'b1;

            next_state = IN_READ;
          // USB write transaction
          end else if (pkt_inbound.pid == PID_OUT
                       && pkt_inbound.addr == DEVICE_ADDR
                       && pkt_inbound.endp == DATA_ENDP) begin
            next_state = OUT_WRITE;
          // Ignore token packet completely if not meant for this device
          end else ;
        end
      end

      IN_READ: begin
        pkt_outbound = '{pid: PID_DATA0, addr: 'x, endp: 'x,
                         payload: payload_outbound};

        if (fake_corrupt != 0) begin
          ready_outbound = 1'b1;
          send_corrupt = 1'b1;
        end else if (fake_timeout != 0) begin
          dec_fake_timeout = 1'b1;

          next_state = DATA0_READ;
        end else begin
          ready_outbound = 1'b1;
        end

        if (done_sending) begin
          if (fake_corrupt != 0) dec_fake_corrupt = 1'b1;

          next_state = DATA0_READ;
        end
      end

      DATA0_READ: begin
        ready_inbound = 1'b1;

        if (done_receiving) begin
          unique if (pkt_inbound.pid == PID_ACK) begin
            device_done_tx = 1'b1;

            clr_failed = 1'b1;

            next_state = PROTO_IDLE;
          end else if (pkt_inbound.pid == PID_NAK) begin
            if (failed == `TX_RETRIES-1) begin
              device_done_tx = 1'b1;
              device_null_tx = 1'b1;

              clr_failed = 1'b1;

              next_state = PROTO_IDLE;
            end else if (fake_timeout != 0) begin
              dec_fake_timeout = 1'b1;

              en_failed = 1'b1;
            end else begin // Legitimate NAK from host
              en_failed = 1'b1;

              next_state = NAK_READ;
            end
          // Ignore non-handshake packet completely
          end else ; // TODO: VCS catches the negedge of done_receiving
        end
      end

      NAK_READ: begin
        ready_outbound = 1'b1;
        if (fake_corrupt != 0) send_corrupt = 1'b1;

        pkt_outbound = '{pid: PID_DATA0, addr: 'x, endp: 'x,
                         payload: payload_outbound};

        if (done_sending) begin
          if (fake_corrupt != 0) dec_fake_corrupt = 1'b1;

          next_state = DATA0_READ;
        end
      end

      OUT_WRITE: begin
        ready_inbound = 1'b1;

        if (done_receiving) begin
          // Send legitimate NAK
          if (recv_error || recv_timeout || pkt_inbound.pid != PID_DATA0) begin
            en_failed = 1'b1;

            next_state = NAK_WRITE;
          end else if (pkt_inbound.pid == PID_DATA0) begin
            if (fake_nak_data != 0) begin
              en_failed = 1'b1;

              next_state = NAK_WRITE;
            end else begin
              ld_payload_inbound = 1'b1;

              clr_failed = 1'b1;

              next_state = DATA0_WRITE;
            end
          end
        end
      end

      DATA0_WRITE: begin
        ready_outbound = 1'b1;

        pkt_outbound = '{pid: PID_ACK, addr: 'x, endp: 'x, payload: 'x};

        if (done_sending) begin
          device_done_tx = 1'b1;

          write_en = 1'b1;

          next_state = PROTO_IDLE;
        end
      end

      NAK_WRITE: begin
        ready_outbound = 1'b1;

        pkt_outbound = '{pid: PID_NAK, addr: 'x, endp: 'x, payload: 'x};

        if (done_sending) begin
          if (failed == `TX_RETRIES-1) begin // Transaction thrown out
            device_done_tx = 1'b1;
            device_null_tx = 1'b1;

            clr_failed = 1'b1;

            next_state = PROTO_IDLE;
          end else if (fake_nak_data != 0) begin // Continue sending fake NAKs
            dec_fake_nak_data = 1'b1;

            next_state = OUT_WRITE;
          end else begin // Continue with normal transaction
            next_state = OUT_WRITE;
          end
        end
      end
    endcase
  end
  `endprotect
endmodule : USBDevice

`protect
module _CRC5 (
    input logic clock, reset_n,
    input logic en, clr,
    input logic serial,
    output logic [4:0] crc5_residue);

  logic feedback;
  logic [4:0] poly;

  assign crc5_residue = poly;
  assign feedback = poly[4] ^ serial;

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) poly <= '1;
    else if (clr) poly <= '1;
    else if (en) poly <= {poly[3:2], poly[1] ^ feedback, poly[0], feedback};
  end
endmodule : _CRC5
`endprotect

`protect
module _CRC16 (
    input logic clock, reset_n,
    input logic en, clr,
    input logic serial,
    output logic [15:0] crc16_residue);

  logic feedback;
  logic [15:0] poly;

  assign crc16_residue = poly;
  assign feedback = poly[15] ^ serial;

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      poly <= '1;
    end else if (clr) begin
      poly <= '1;
    end else if (en) begin
      poly <= {poly[14] ^ feedback, poly[13:2], poly[1] ^ feedback, poly[0],
              feedback};
    end
  end
endmodule : _CRC16
`endprotect

`protect
module _BitStuffer (
    input logic  clock, reset_n,
    input logic en, serial_in,
    output logic wait_stuffing, serial_out);

  logic [2:0] ones_seen;

  assign wait_stuffing = ones_seen == `STUFF_LENGTH-1;
  assign serial_out = wait_stuffing ? 1'b0 : serial_in;

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      ones_seen <= '0;
    end else if (~en) begin
      ones_seen <= '0;
    end else begin
      if (~serial_in) ones_seen <= '0;
      else if (ones_seen == `STUFF_LENGTH-1) ones_seen <= '0;
      else ones_seen <= ones_seen + 1;
    end
  end
endmodule : _BitStuffer
`endprotect

`protect
module _NRZIEncode (
    USBWires wires,
    input logic clock, reset_n,
    input logic en,
    input logic serial_se0, serial_out);

  usb_bus_t current_bus, prev_bus;

  assign {wires.DP, wires.DM} = current_bus;

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) prev_bus <= USB_J;
    else if (en) prev_bus <= current_bus;
  end

  always_comb begin
    current_bus = USB_NC;

    if (en) begin
      if (serial_se0) current_bus = USB_SE0;
      else begin
        unique case (prev_bus)
          USB_J: current_bus = serial_out ? USB_J : USB_K;
          USB_K: current_bus = serial_out ? USB_K : USB_J;
          USB_SE0: current_bus = serial_out ? USB_J : USB_K;
        endcase
      end
    end
  end
endmodule : _NRZIEncode
`endprotect

`protect
module _NRZIDecode (
    USBWires wires,
    input logic clock, reset_n,
    input logic en,
    output logic serial_se0, serial_in);

  usb_bus_t current_bus, prev_bus;

  assign current_bus = usb_bus_t'({wires.DP, wires.DM});
  assign serial_se0 = current_bus == USB_SE0;

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) prev_bus <= USB_J;
    else if (en) prev_bus <= current_bus;
  end

  always_comb begin
    serial_in = 1'b0; // Edge-case for analyzer when SE0 for extended periods

    if (en & ~serial_se0) begin
      unique case (prev_bus)
        USB_J: serial_in = current_bus == USB_J;
        USB_K: serial_in = current_bus == USB_K;
        USB_SE0: serial_in = current_bus == USB_J;
      endcase
    end
  end
endmodule : _NRZIDecode
`endprotect

`protect
module _USBDeviceRecv (
    USBWires wires,
    input logic clock, reset_n,
    input logic ready,
    output logic done_receiving,
    output pkt_t pkt_inbound,
    output logic recv_error, recv_timeout);

  logic en_stuff, en_nrzi, en_crc, clr_crc;
  logic [15:0] crc16_residue;
  logic [4:0] crc5_residue;
  logic en_pkt_inbound, clr_pkt_inbound;
  logic ld_pid, ld_addr, ld_endp, ld_payload;
  logic wait_stuffing;

  logic serial_raw, serial_in, serial_se0;

  logic [5:0] ticker;
  logic inc_ticker, clr_ticker;
  enum {
      IDLE, SYNC, PID, PID_N, ADDR, ENDP, PAYLOAD, CRC5, CRC16, EOP_HAND,
      EOP_TOKEN, EOP_DATA
  } current_state, next_state;

  _NRZIDecode
    nrzi_inst (.wires,
               .clock, .reset_n,
               .en(en_nrzi),
               .serial_se0, .serial_in);

  _BitStuffer
    stuff_inst (.clock, .reset_n,
                .en(en_stuff), .serial_in,
                .wait_stuffing, .serial_out(serial_raw));

  _CRC5
    crc5_inst (.clock, .reset_n,
               .en(en_crc & ~wait_stuffing), .clr(clr_crc),
               .serial(serial_raw), .crc5_residue);

  _CRC16
    crc16_inst (.clock, .reset_n,
                .en(en_crc & ~wait_stuffing), .clr(clr_crc),
                .serial(serial_raw), .crc16_residue);

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      ticker <= '0;
    end else if (~wait_stuffing) begin
      if (clr_ticker) ticker <= '0;
      else if (inc_ticker) ticker <= ticker + 1;
    end
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      pkt_inbound <= '{pid: PID_X, addr: 'x, endp: 'x, payload: 'x};
    end else if (~wait_stuffing) begin
      if (clr_pkt_inbound) begin
        pkt_inbound <= '{pid: PID_X, addr: 'x, endp: 'x, payload: 'x};
      end else if (en_pkt_inbound) begin
        unique if (ld_pid) begin
          pkt_inbound.pid <= {serial_raw, pkt_inbound.pid[3:1]};
        end else if (ld_addr) begin
          pkt_inbound.addr <= {serial_raw, pkt_inbound.addr[6:1]};
        end else if (ld_endp) begin
          pkt_inbound.endp <= {serial_raw, pkt_inbound.endp[3:1]};
        end else if (ld_payload) begin
          pkt_inbound.payload <= {serial_raw, pkt_inbound.payload[63:1]};
        end else ; // unique0
      end
    end
  end

    always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) current_state <= IDLE;
    else if (~wait_stuffing) current_state <= next_state;
  end

  always_comb begin
    done_receiving = 1'b0;
    recv_error = 1'b0;
    recv_timeout = 1'b0;

    en_stuff = 1'b0;
    en_nrzi = 1'b1; // Special-use only
    en_crc = 1'b0;
    clr_crc = 1'b0;
    en_pkt_inbound = 1'b0;
    clr_pkt_inbound = 1'b0;
    ld_pid = 1'b0;
    ld_addr = 1'b0;
    ld_endp = 1'b0;
    ld_payload = 1'b0;

    inc_ticker = 1'b0;
    clr_ticker = 1'b0;
    next_state = current_state;

    unique case (current_state)
      IDLE: begin
        if (ready) begin
          if (~serial_se0 & ~serial_raw) begin
            clr_ticker = 1'b1;

            next_state = SYNC;
          end else if (ticker == `TIMEOUT-1) begin
            done_receiving = 1'b1;
            recv_timeout = 1'b1;

            clr_ticker = 1'b1;
          end else begin
            inc_ticker = 1'b1;
          end
        end
      end

      // Last tick of IDLE captures first bit of SYNC, so last bit happens when
      // ticker == `SYNC_BITS-2 not `SYNC_BITS-1
      SYNC: begin
        if (ticker == `SYNC_BITS-2) begin
          clr_ticker = 1'b1;
          next_state = PID;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      PID: begin
        en_pkt_inbound = 1'b1;
        ld_pid = 1'b1;

        if (ticker == `PID_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = PID_N;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      PID_N: begin
        if (ticker == `PID_BITS-1) begin
          clr_ticker = 1'b1;

          unique case (pkt_inbound.pid)
            PID_OUT, PID_IN: next_state = ADDR;
            PID_DATA0: next_state = PAYLOAD;
            PID_ACK, PID_NAK: next_state = EOP_HAND;
          endcase
        end else begin
          inc_ticker = 1'b1;
        end
      end

      ADDR: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;
        en_pkt_inbound = 1'b1;
        ld_addr = 1'b1;

        if (ticker == `ADDR_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = ENDP;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      ENDP: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;
        en_pkt_inbound = 1'b1;
        ld_endp = 1'b1;

        if (ticker == `ENDP_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = CRC5;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      PAYLOAD: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;
        en_pkt_inbound = 1'b1;
        ld_payload = 1'b1;

        if (ticker == `PAYLOAD_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = CRC16;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      CRC5: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;

        if (ticker == `CRC5_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = EOP_TOKEN;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      CRC16: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;

        if (ticker == `CRC16_BITS-1) begin
          clr_ticker = 1'b1;

          next_state = EOP_DATA;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      EOP_HAND: begin
        if (ticker == `EOP_BITS-1) begin
          done_receiving = 1'b1;

          clr_pkt_inbound = 1'b1;

          clr_ticker = 1'b1;
          next_state = IDLE;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      EOP_TOKEN: begin
        if (ticker == `EOP_BITS-1) begin
          done_receiving = 1'b1;
          recv_error = crc5_residue != `CRC5_RESIDUE;

          clr_crc = 1'b1;
          clr_pkt_inbound = 1'b1;

          clr_ticker = 1'b1;
          next_state = IDLE;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      EOP_DATA: begin
        if (ticker == `EOP_BITS-1) begin
          done_receiving = 1'b1;
          recv_error = crc16_residue != `CRC16_RESIDUE;

          clr_crc = 1'b1;
          clr_pkt_inbound = 1'b1;

          clr_ticker = 1'b1;
          next_state = IDLE;
        end else begin
          inc_ticker = 1'b1;
        end
      end
    endcase
  end
endmodule : _USBDeviceRecv
`endprotect

`protect
module _USBDeviceSend (
    USBWires wires,
    input logic clock, reset_n,
    input logic ready,
    output logic done_sending,
    input pkt_t pkt_outbound,
    input logic send_corrupt);

  logic en_stuff, en_nrzi, en_crc, clr_crc;
  logic [15:0] crc16_residue;
  logic [4:0] crc5_residue;
  logic wait_stuffing;

  logic serial_raw, serial_out, serial_se0;

  logic [7:0] latency_ticker;
  logic en_latency_ticker, clr_latency_ticker;
  logic [5:0] ticker;
  logic inc_ticker, clr_ticker;
  enum {
      IDLE, SYNC, PID, PID_N, ADDR, ENDP, PAYLOAD, CRC5, CRC16, EOP
  } current_state, next_state;

  _NRZIEncode
    nrzi_inst (.wires,
               .clock, .reset_n,
               .en(en_nrzi),
               .serial_se0, .serial_out);

  _BitStuffer
    stuff_inst (.clock, .reset_n,
                .en(en_stuff), .serial_in(serial_raw),
                .wait_stuffing, .serial_out);

  _CRC5
    crc5_inst (.clock, .reset_n,
               .en(en_crc & ~wait_stuffing), .clr(clr_crc),
               .serial(serial_raw), .crc5_residue);

  _CRC16
    crc16_inst (.clock, .reset_n,
                .en(en_crc & ~wait_stuffing), .clr(clr_crc),
                .serial(serial_raw), .crc16_residue);

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) latency_ticker <= $urandom_range(50);
    else if (clr_latency_ticker) latency_ticker <= $urandom_range(50);
    else if (en_latency_ticker) latency_ticker <= latency_ticker - 1;
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      ticker <= '0;
    end else if (~wait_stuffing) begin
      if (clr_ticker) ticker <= '0;
      else if (inc_ticker) ticker <= ticker + 1;
    end
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) current_state <= IDLE;
    else if (~wait_stuffing) current_state <= next_state;
  end

  always_comb begin
    done_sending = 1'b0;

    en_stuff = 1'b0;
    en_nrzi = 1'b1; // Special-use only (IDLE)
    en_crc = 1'b0;
    clr_crc = 1'b0;

    serial_se0 = 1'b0;
    serial_raw = 1'bx;

    en_latency_ticker = 1'b0;
    clr_latency_ticker = 1'b0;
    inc_ticker = 1'b0;
    clr_ticker = 1'b0;
    next_state = current_state;

    unique case (current_state)
      IDLE: begin
        en_nrzi = 1'b0; // Just in case USBHost does something strange

        if (ready) begin
          if (latency_ticker == 0) begin
            clr_latency_ticker = 1'b1;
            next_state = SYNC;
          end else begin
            en_latency_ticker = 1'b1;
          end
        end
      end

      SYNC: begin
        if (ticker == `SYNC_BITS-1) begin
          serial_raw = 1'b1;

          clr_ticker = 1'b1;
          next_state = PID;
        end else begin
          serial_raw = 1'b0;

          inc_ticker = 1'b1;
        end
      end

      PID: begin
        serial_raw = pkt_outbound.pid[ticker];

        if (ticker == `PID_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = PID_N;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      PID_N: begin
        serial_raw = ~pkt_outbound.pid[ticker];

        if (ticker == `PID_BITS-1) begin
          clr_ticker = 1'b1;

          unique case (pkt_outbound.pid)
            PID_OUT, PID_IN: next_state = ADDR;
            PID_DATA0: next_state = PAYLOAD;
            PID_ACK, PID_NAK: next_state = EOP;
          endcase
        end else begin
          inc_ticker = 1'b1;
        end
      end

      ADDR: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;

        serial_raw = pkt_outbound.addr[ticker];

        if (ticker == `ADDR_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = ENDP;
        end begin
          inc_ticker = 1'b1;
        end
      end

      ENDP: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;

        serial_raw = pkt_outbound.endp[ticker];

        if (ticker == `ENDP_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = CRC5;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      PAYLOAD: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;

        serial_raw = pkt_outbound.payload[ticker];

        if (ticker == `PAYLOAD_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = CRC16;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      CRC5: begin
        en_stuff = 1'b1;

        if(send_corrupt) serial_raw = $urandom_range(1);
        else serial_raw = ~crc5_residue[(`CRC5_BITS-1) - ticker];

        if (ticker == `CRC5_BITS-1) begin
          clr_crc = 1'b1;

          clr_ticker = 1'b1;
          next_state = EOP;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      CRC16: begin
        en_stuff = 1'b1;

        if(send_corrupt) serial_raw = $urandom_range(1);
        else serial_raw = ~crc16_residue[(`CRC16_BITS-1) - ticker];

        if (ticker == `CRC16_BITS-1) begin
          clr_crc = 1'b1;

          clr_ticker = 1'b1;
          next_state = EOP;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      EOP: begin
        if (ticker == `EOP_BITS-1) begin
          done_sending = 1'b1;

          serial_raw = 1'b1;

          clr_ticker = 1'b1;
          next_state = IDLE;
        end else begin
          serial_se0 = 1'b1;
          serial_raw = 1'b1;

          inc_ticker = 1'b1;
        end
      end
    endcase
  end
endmodule : _USBDeviceSend
`endprotect

module _USBAnalyzer #(parameter DEVICE_ADDR=`DEVICE_ADDR, DATA_ENDP=`DATA_ENDP,
                                ADDR_ENDP=`ADDR_ENDP) (
    USBWires wires,
    input logic clock, reset_n,
    output debug_pkt_t debug_pkt_ext);

  default clocking cb_main @(posedge clock); endclocking

  `protect
  function logic [4:0] fn_crc5_nr(debug_pkt_t debug_pkt);
    logic feedback;
    logic [4:0] poly;

    poly = '1;

    for (int i = 0; i < `ADDR_BITS; i++) begin
      feedback = poly[4] ^ debug_pkt.addr[i];
      poly = {poly[3:2], poly[1] ^ feedback, poly[0], feedback};
    end

    for (int i = 0; i < `ENDP_BITS; i++) begin
      feedback = poly[4] ^ debug_pkt.endp[i];
      poly = {poly[3:2], poly[1] ^ feedback, poly[0], feedback};
    end

    poly = ~poly;
    poly = {<<{poly}};

    return poly;
  endfunction

  function logic [15:0] fn_crc16_nr(debug_pkt_t debug_pkt);
    logic feedback;
    logic [15:0] poly;

    poly = '1;

    for (int i = 0; i < `PAYLOAD_BITS; i++) begin
      feedback = poly[15] ^ debug_pkt.payload[i];
      poly = {poly[14] ^ feedback, poly[13:2], poly[1] ^ feedback, poly[0],
              feedback};
    end

    poly = ~poly;
    poly = {<<{poly}};

    return poly;
  endfunction
  `endprotect

  function string fn_encode_bus(usb_bus_t usb_buses[]);
    string encoded;
    string temp;

    encoded = "";
    temp = "";

    foreach (usb_buses[i]) begin
      unique case (usb_buses[i])
        USB_J: temp = "J";
        USB_K: temp = "K";
        USB_SE0: temp = "0";
        USB_SE1: temp = "1";
      endcase

      encoded = {encoded, temp};
    end
    return encoded;
  endfunction

  usb_bus_t current_bus;
  assign current_bus = usb_bus_t'({wires.DP, wires.DM});

  `protect
  logic done_receiving, recv_error;

  logic en_stuff, en_nrzi, en_crc, clr_crc;
  logic [15:0] crc16_residue;
  logic [4:0] crc5_residue;
  logic en_pkt_inbound, clr_pkt_inbound;
  debug_pkt_t debug_pkt;
  logic ld_sync, ld_pid, ld_pid_n, ld_addr, ld_endp, ld_payload, ld_crc5,
        ld_crc16, chk_eop;
  logic wait_stuffing;

  logic serial_raw, serial_in, serial_se0;

  logic [5:0] ticker;
  logic inc_ticker, clr_ticker;
  enum {
      IDLE, SYNC, PID, PID_N, ADDR, ENDP, PAYLOAD, CRC5, CRC16, EOP_HAND,
      EOP_TOKEN, EOP_DATA
  } current_state, next_state;

  assign debug_pkt_ext = debug_pkt;

  _USBTerminal
    usb_terminal_inst (.clock,
                       .wait_stuffing, .en_pkt_inbound, .serial_se0,
                       .serial_raw,
                       .debug_pkt,
                       .current_bus);

  _NRZIDecode
    nrzi_inst (.wires,
               .clock, .reset_n,
               .en(en_nrzi),
               .serial_se0, .serial_in(serial_in));

  _BitStuffer
    stuff_inst (.clock, .reset_n,
                .en(en_stuff), .serial_in,
                .wait_stuffing, .serial_out(serial_raw));

  _CRC5
    crc5_inst (.clock, .reset_n,
               .en(en_crc & ~wait_stuffing), .clr(clr_crc),
               .serial(serial_raw), .crc5_residue);

  _CRC16
    crc16_inst (.clock, .reset_n,
                .en(en_crc & ~wait_stuffing), .clr(clr_crc),
                .serial(serial_raw), .crc16_residue);

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      ticker <= '0;
    end else if (~wait_stuffing) begin
      if (clr_ticker) ticker <= '0;
      else if (inc_ticker) ticker <= ticker + 1;
    end
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      debug_pkt <= '{sync: 'x, pid: 'x, pid_n: 'x, addr: 'x, endp: 'x,
                     payload: 'x, crc5: 'x, crc16: 'x};
    end else if (~wait_stuffing) begin
      if (clr_pkt_inbound) begin
        debug_pkt <= '{sync: 'x, pid: 'x, pid_n: 'x, addr: 'x, endp: 'x,
                       payload: 'x, crc5: 'x, crc16: 'x};
      end else if (en_pkt_inbound) begin
        unique if (ld_sync) begin
          debug_pkt.sync <= {debug_pkt.sync[6:0], serial_raw};
        end else if (ld_pid) begin
          debug_pkt.pid <= {serial_raw, debug_pkt.pid[3:1]};
        end else if (ld_pid_n) begin
          debug_pkt.pid_n <= {serial_raw, debug_pkt.pid_n[3:1]};
        end else if (ld_addr) begin
          debug_pkt.addr <= {serial_raw, debug_pkt.addr[6:1]};
        end else if (ld_endp) begin
          debug_pkt.endp <= {serial_raw, debug_pkt.endp[3:1]};
        end else if (ld_payload) begin
          debug_pkt.payload <= {serial_raw, debug_pkt.payload[63:1]};
        end else if (ld_crc5) begin
          debug_pkt.crc5 <= {serial_raw, debug_pkt.crc5[4:1]};
        end else if (ld_crc16) begin
          debug_pkt.crc16 <= {serial_raw, debug_pkt.crc16[15:1]};
        end else ; // unique0
      end
    end
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) current_state <= IDLE;
    else if (~wait_stuffing) current_state <= next_state;
  end

  always_comb begin
    done_receiving = 1'b0;
    recv_error = 1'b0;

    en_stuff = 1'b0;
    en_nrzi = 1'b1; // Special-use only
    en_crc = 1'b0;
    clr_crc = 1'b0;
    en_pkt_inbound = 1'b0;
    clr_pkt_inbound = 1'b0;
    ld_sync = 1'b0;
    ld_pid = 1'b0;
    ld_pid_n = 1'b0;
    ld_addr = 1'b0;
    ld_endp = 1'b0;
    ld_payload = 1'b0;
    ld_crc5 = 1'b0;
    ld_crc16 = 1'b0;
    chk_eop = 1'b0;

    inc_ticker = 1'b0;
    clr_ticker = 1'b0;
    next_state = current_state;

    unique case (current_state)
      IDLE: begin
        if (~serial_se0 & ~serial_raw) begin
          en_pkt_inbound = 1'b1;
          ld_sync = 1'b1;

          next_state = SYNC;
        end
      end

      // Last tick of IDLE captures first bit of SYNC, so last bit happens when
      // ticker == `SYNC_BITS-2 not `SYNC_BITS-1
      SYNC: begin
        en_pkt_inbound = 1'b1;
        ld_sync = 1'b1;

        if (ticker == `SYNC_BITS-2) begin
          clr_ticker = 1'b1;
          next_state = PID;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      PID: begin
        en_pkt_inbound = 1'b1;
        ld_pid = 1'b1;

        if (ticker == `PID_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = PID_N;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      PID_N: begin
        en_pkt_inbound = 1'b1;
        ld_pid_n = 1'b1;

        if (ticker == `PID_BITS-1) begin
          clr_ticker = 1'b1;

          unique case (debug_pkt.pid)
            PID_OUT, PID_IN: next_state = ADDR;
            PID_DATA0: next_state = PAYLOAD;
            PID_ACK, PID_NAK: next_state = EOP_HAND;
          endcase
        end else begin
          inc_ticker = 1'b1;
        end
      end

      ADDR: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;
        en_pkt_inbound = 1'b1;
        ld_addr = 1'b1;

        if (ticker == `ADDR_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = ENDP;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      ENDP: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;
        en_pkt_inbound = 1'b1;
        ld_endp = 1'b1;

        if (ticker == `ENDP_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = CRC5;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      PAYLOAD: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;
        en_pkt_inbound = 1'b1;
        ld_payload = 1'b1;

        if (ticker == `PAYLOAD_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = CRC16;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      CRC5: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;
        en_pkt_inbound = 1'b1;
        ld_crc5 = 1'b1;

        if (ticker == `CRC5_BITS-1) begin
          clr_ticker = 1'b1;
          next_state = EOP_TOKEN;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      CRC16: begin
        en_stuff = 1'b1;
        en_crc = 1'b1;
        en_pkt_inbound = 1'b1;
        ld_crc16 = 1'b1;

        if (ticker == `CRC16_BITS-1) begin
          clr_ticker = 1'b1;

          next_state = EOP_DATA;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      EOP_HAND: begin
        // Edge case
        if (wait_stuffing) en_pkt_inbound = 1'b1;
        else chk_eop = 1'b1;

        if (ticker == `EOP_BITS-1) begin
          done_receiving = 1'b1;

          clr_pkt_inbound = 1'b1;

          clr_ticker = 1'b1;
          next_state = IDLE;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      EOP_TOKEN: begin
        // Edge case
        if (wait_stuffing) en_pkt_inbound = 1'b1;
        else chk_eop = 1'b1; // Edge case

        if (ticker == `EOP_BITS-1) begin
          done_receiving = 1'b1;
          recv_error = crc5_residue != `CRC5_RESIDUE;

          clr_crc = 1'b1;
          clr_pkt_inbound = 1'b1;

          clr_ticker = 1'b1;
          next_state = IDLE;
        end else begin
          inc_ticker = 1'b1;
        end
      end

      EOP_DATA: begin
        if (~wait_stuffing) chk_eop = 1'b1; // Edge case

        if (ticker == `EOP_BITS-1) begin
          done_receiving = 1'b1;
          recv_error = crc16_residue != `CRC16_RESIDUE;

          clr_crc = 1'b1;
          clr_pkt_inbound = 1'b1;

          clr_ticker = 1'b1;
          next_state = IDLE;
        end else begin
          inc_ticker = 1'b1;
        end
      end
    endcase
  end
  `endprotect

  /*--------------------------------------------------------------------------
   * Concurrent Assertions for USB Protocol
   *--------------------------------------------------------------------------*/
  chk_valid_bus: assert property (
    disable iff (~reset_n)

    `TRUE |-> current_bus inside {valid_usb_buses}
  ) else begin
    $error("!!! %m: USB analyzer detected invalid bus state DP=%b DM=%b",
           wires.DP, wires.DM);
  end

  chk_bus_turnaround: assert property (
    disable iff (~reset_n)

    $rose(chk_eop) |-> ##3
      current_bus == USB_SE0
  ) else $fatal("!!! %m: USB analyzer detected no bus turnaround cycle");

  chk_stuffing: assert property (
    disable iff (~reset_n)

    (en_stuff & serial_raw) [*6] |=>
      ~serial_raw
  ) else $error("!!! %m: USB analyzer did not detect stuffed bit");

  chk_sync: assert property (
    disable iff (~reset_n)

    $rose(ld_sync) |-> ##(`SYNC_BITS)
      debug_pkt.sync == `SYNC
  ) else begin
    $fatal("!!! %m: USB analyzer detected invalid SYNC %b", debug_pkt.sync);
  end

  chk_pid: assert property (
    disable iff (~reset_n)

    $rose(ld_pid) |-> ##(`PID_BITS)
      debug_pkt.pid inside {valid_pids}
  ) else begin
    $error("!!! %m: USB analyzer detected invalid PID %b", debug_pkt.pid);
  end

  chk_pid_n: assert property (
    disable iff (~reset_n)

    $rose(ld_pid_n) |-> ##(`PID_BITS)
      debug_pkt.pid_n == ~debug_pkt.pid
  ) else begin
    $error("!!! %m: USB analyzer detected invalid PID %b PID_N %b",
           debug_pkt.pid, debug_pkt.pid_n);
  end

  chk_addr: assert property (
    disable iff (~reset_n)

    $rose(ld_addr) |-> ##(`ADDR_BITS)
      debug_pkt.addr == `DEVICE_ADDR
  ) else begin
    $error({"!!! %m: USB analyzer detected token packet with invalid device ",
            "address %0d"},
           debug_pkt.addr);
  end

  chk_crc5: assert property (
    disable iff (~reset_n)

    $rose(ld_crc5) ##1 $fell(ld_crc5) [->1] |->
      debug_pkt.crc5 == fn_crc5_nr(debug_pkt)
  ) else begin
    $error({"!!! %m: USB analyzer detected token packet with invalid CRC5 %h ",
            "(correct %h)"},
           debug_pkt.crc5, fn_crc5_nr(debug_pkt));
  end

  chk_crc16: assert property (
    disable iff (~reset_n)

    $rose(ld_crc16) ##1 $fell(ld_crc16) [->1] |->
      debug_pkt.crc16 == fn_crc16_nr(debug_pkt)
  ) else begin
    $error({"!!! %m: USB analyzer detected data packet with invalid CRC16 %h ",
            "(correct %h)"},
           debug_pkt.crc16, fn_crc16_nr(debug_pkt));
  end

  chk_eop_bus: assert property (
    disable iff (~reset_n)

    $rose(chk_eop) |->
      current_bus == USB_SE0 ##1
      current_bus == USB_SE0 ##1
      current_bus == USB_J
  ) else $fatal("!!! %m: USB analyzer detected invalid EOP");

  /*--------------------------------------------------------------------------
   * Full USB Protocol Assertion
   *   seq_error: Corrupt DATA0 followed by NAK
   *   seq_out_timetou: Legitimate timeout on DATA0 during OUT transaction
   *   seq_in_timetou: Legitimate timeout on DATA0 during IN transaction
   *
   *   Sequence repetition operators:
   *     ->#: GOTO repetition; may be false for an arbitrary number of cycles,
   *          then matches on the last instance
   *     *#: CONSECUTIVE repetition; must be true for a specified number of
   *         cycles
   *   Sequence events:
   *     $rose( ): POSEDGE; only match on the first assertion of a signal
   *     $fell( ): NEGEDGE; only match on the first de-assertion of a signal
   *--------------------------------------------------------------------------*/
  sequence seq_error;
    $fell(en_pkt_inbound) [->1] ##[`EOP_BITS-1:`EOP_BITS]
    $rose(done_receiving) && recv_error && debug_pkt.pid == PID_DATA0 ##1
    $fell(en_pkt_inbound) [->1] ##(`EOP_BITS-1)
    $rose(done_receiving) && debug_pkt.pid == PID_NAK;
  endsequence

  sequence seq_out_timeout;
    $fell(en_pkt_inbound) [->1] ##[`EOP_BITS-1:`EOP_BITS]
    $rose(done_receiving) && ~recv_error && debug_pkt.pid == PID_DATA0 ##1
    current_bus == USB_SE0 [*(`TIMEOUT)];
  endsequence

  sequence seq_in_timeout;
    current_bus == USB_SE0 [*(`TIMEOUT-1):(2*`TIMEOUT-1)] ##1
    $fell(en_pkt_inbound) [->1] ##(`EOP_BITS-1)
    $rose(done_receiving) && debug_pkt.pid == PID_NAK;
  endsequence

  chk_protocol: assert property (
    disable iff (~reset_n)

    /*
     * Read/write transaction always starts with an OUT transaction to provide
     * the memory address
     */
    $rose(done_receiving) && ~recv_error && debug_pkt.pid == PID_OUT
                          && debug_pkt.addr == `DEVICE_ADDR
                          && debug_pkt.endp == `ADDR_ENDP |=>
      // OUT ADDR transaction
      (seq_error or seq_out_timeout) [*0:7] ##1
      $fell(en_pkt_inbound) [->1] ##[`EOP_BITS-1:`EOP_BITS]
      $rose(done_receiving) && ~recv_error && debug_pkt.pid == PID_DATA0
                            && debug_pkt.payload[47:0] == '0 ##1
      $fell(en_pkt_inbound) [->1] ##(`EOP_BITS-1)
      $rose(done_receiving) && debug_pkt.pid == PID_ACK ##1

      // OUT or IN DATA transaction
      $fell(en_pkt_inbound) [->1] ##(`EOP_BITS-1)
      $rose(done_receiving) && ~recv_error
                            && debug_pkt.pid inside {PID_IN, PID_OUT}
                            && debug_pkt.endp == `DATA_ENDP ##1
      (seq_error or seq_in_timeout or seq_out_timeout) [*0:7] ##1
      $fell(en_pkt_inbound) [->1] ##[`EOP_BITS-1:`EOP_BITS]
      $rose(done_receiving) && ~recv_error && debug_pkt.pid == PID_DATA0 ##1
      $fell(en_pkt_inbound) [->1] ##(`EOP_BITS-1)
      $rose(done_receiving) && debug_pkt.pid == PID_ACK
  ) else begin
    $error("!!! %m: USB analyzer detected invalid sequence of packets");
  end
endmodule : _USBAnalyzer

program _USBTerminal (
  input logic clock,
  input logic wait_stuffing, en_pkt_inbound, serial_se0, serial_raw,
  input debug_pkt_t debug_pkt,
  input usb_bus_t current_bus);

  default clocking cb_main @(posedge clock); endclocking

  usb_bus_t debug_usb_bus[$];

  integer debug_level = 0;
  initial $value$plusargs("VERBOSE=%d", debug_level);

  initial begin
    string bitstream_str;

    forever begin
      debug_usb_bus = '{};
      bitstream_str = "";

      @(posedge en_pkt_inbound);
      ##1 ;
      $display({"=================================================\n",
                "%m @%0t: USB analyzer is interpreting a new packet"}, $time);

      while (en_pkt_inbound) begin
        debug_usb_bus.push_back(current_bus);

        if (wait_stuffing) begin
          if (serial_raw) bitstream_str = {bitstream_str, "?1"};
          else bitstream_str = {bitstream_str, " "};
        end else begin
          if (~serial_se0 & ~serial_raw) begin
            bitstream_str = {bitstream_str, "0"};
          end else if (~serial_se0 & serial_raw) begin
            bitstream_str = {bitstream_str, "1"};
          end
        end

        ##1 ;
      end

      repeat(`EOP_BITS-1) begin
        debug_usb_bus.push_back(current_bus);
        ##1 ;
      end
      debug_usb_bus.push_back(current_bus);

      if (debug_level > 2) begin
        $display("Raw Bus Traffic: %s", fn_encode_bus(debug_usb_bus));
      end
      if (debug_level > 1) begin
        $display("Bitstream:       %s", bitstream_str);
      end
      if (debug_level == 3) begin
        $display("Packet: %p", debug_pkt);
      end

      unique case (pid_t'(debug_pkt.pid))
        PID_OUT: begin
          $display({"%m @%0t: USB analyzer detected *OUT* with ADDR=%0d ",
                    "ENDP=%0d CRC5=%x (correct %x)"}, $time, debug_pkt.addr,
                    debug_pkt.endp, debug_pkt.crc5, fn_crc5_nr(debug_pkt));
        end

        PID_IN: begin
          $display({"%m @%0t: USB analyzer detected *IN* with ADDR=%0d ",
                    "ENDP=%0d CRC5=%x (correct %x)"}, $time, debug_pkt.addr,
                    debug_pkt.endp, debug_pkt.crc5, fn_crc5_nr(debug_pkt));
        end

        PID_DATA0: begin
          $display({"%m @%0t: USB analyzer detected *DATA0* with PAYLOAD=%x ",
                    "CRC16=%x (correct %x)"}, $time, debug_pkt.payload,
                    debug_pkt.crc16, fn_crc16_nr(debug_pkt));
        end

        PID_ACK: begin
          $display("%m @%0t: USB analyzer detected *ACK*", $time);
        end

        PID_NAK: begin
          $display("%m @%0t: USB analyzer detected *NAK*", $time);
        end

        default: begin
          $display("%m @%0t: USBAnalyzer could not interpret packet", $time);
        end
      endcase

      $display("=================================================\n\n");
      ##1 ;
    end
  end
endprogram : _USBTerminal