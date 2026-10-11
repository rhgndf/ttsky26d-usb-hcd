/*
 * usb_sie.v - oversampled USB full-speed PHY and basic host/device SIE
 * All line I/O is synchronous to the 48 MHz system clock.
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module usb_sie (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        dp_in,
    input  wire        dm_in,
    input  wire [5:0]  ctrl,
    input  wire [6:0]  addr_reg,
    input  wire [3:0]  cmd_ep,
    input  wire [1:0]  cmd_op,
    input  wire        cmd_data1,
    input  wire        cmd_any,
    input  wire        cmd_wr,
    input  wire [3:0]  len_reg,
    input  wire [7:0]  buf_rdata,
    output reg         busy,
    output reg         dp_sync,
    output reg         dm_sync,
    output reg         se0_long,
    output reg  [10:0] frame,
    output wire        buf_we,
    output wire [7:0]  buf_wdata,
    output wire        buf_inc,
    output reg         bufptr_clr,
    output reg         op_done,
    output reg         res_wr,
    output reg  [7:0]  res_val,
    output reg         tok_wr,
    output reg  [7:0]  tok_val,
    output reg         len_wr,
    output reg  [3:0]  len_val,
    output reg  [3:0]  irq_set,
    output wire        tx_dp,
    output wire        tx_dm,
    output wire        tx_oe,
    output wire        tx_active
);

    localparam TX_IDLE=2'd0, TX_BITS=2'd1, TX_EOP0=2'd2, TX_EOPJ=2'd3;
    localparam H_IDLE=4'd0, H_COPY=4'd1, H_TOKEN=4'd2, H_DATA=4'd3,
               H_WAIT=4'd4, H_RXCOPY=4'd5, H_ACK=4'd6, H_SOF=4'd7;
    localparam D_IDLE=3'd0, D_WAITDATA=3'd1, D_RXCOPY=3'd2,
               D_TXCOPY=3'd3, D_WAITACK=3'd4, D_HANDSHAKE=3'd5;

    reg [1:0] dp_s, dm_s;
    reg [1:0] rx_phase;
    reg [7:0] se0_count;
    reg [7:0] line_idle_count;
    reg line_sample_dp, line_sample_dm;
    reg line_reported_dp, line_reported_dm;
    reg old_se0_long;
    reg [15:0] sof_timer;
    reg sof_pending;
    reg [3:0] host_state;
    reg [2:0] dev_state;
    reg [1:0] tx_state;
    reg [2:0] tx_phase;
    reg [6:0] tx_bit_index;
    reg [6:0] timeout_count;
    reg rx_line_dp, rx_line_dm;
    reg [4:0] tx_eop_count;
    reg [3:0] tx_ones;
    reg tx_stuff;
    reg tx_line_dp, tx_line_dm;
    reg tx_done;
    reg [7:0] tx_mem [0:15];
    reg [6:0] tx_nbits;
    reg [3:0] tx_payload_len;
    reg [7:0] tx_payload [0:7];
    reg [3:0] host_op;
    reg [3:0] host_ep;
    reg host_data1;
    reg host_is_setup;
    reg [3:0] copy_index;
    reg [3:0] rx_copy_len;
    reg [3:0] rx_copy_index;
    reg [3:0] tx_copy_len;
    reg [3:0] tx_copy_index;
    reg [7:0] rx_payload [0:7];
    reg [3:0] rx_payload_len;
    reg [2:0] dev_op;
    reg [3:0] dev_ep;
    reg dev_any;
    reg dev_data1;
    reg [6:0] rx_addr;
    reg [3:0] rx_ep;
    reg [3:0] rx_pid;
    reg [3:0] rx_error;
    reg [7:0] rx_mem [0:15];
    reg [4:0] rx_byte_count;
    reg [2:0] rx_bit_count;
    reg [7:0] rx_shift;
    reg [3:0] rx_ones;
    reg rx_active;
    reg rx_last_dp, rx_last_dm;
    reg rx_eop;
    reg rx_bad_stuff;
    reg [2:0] rx_eop_se0;
    reg rx_bits_en;
    reg rx_packet_done;
    reg [3:0] pending_handshake;
    reg pending_op_complete;
    reg [3:0] pending_result_pid;
    reg [3:0] pending_result_err;
    reg [3:0] pending_rx_pid;
    reg pending_rx_data1;
    reg pending_valid;

    integer i;
    integer j;
    reg [15:0] crc16_tmp;
    reg [4:0] crc5_tmp;
    reg [10:0] token_tmp;
    reg [7:0] tx_raw_bit;
    reg tx_bit;
    reg tx_dp_i, tx_dm_i;

    function [15:0] crc16_bytes;
        input [3:0] n;
        integer x, y;
        reg [15:0] c;
        reg mix;
        begin
            c = 16'hffff;
            for (x=0; x<8; x=x+1) begin
                if (x<n) begin
                    for (y=0; y<8; y=y+1) begin
                        mix = c[0] ^ tx_payload[x][y];
                        c = c >> 1;
                        if (mix) c = c ^ 16'ha001;
                    end
                end
            end
            crc16_bytes = ~c;
        end
    endfunction

    function [4:0] crc5_token;
        input [10:0] token_data;
        integer x;
        reg [4:0] c;
        reg mix;
        begin
            c = 5'h1f;
            for (x=0; x<11; x=x+1) begin
                mix = c[0] ^ token_data[x];
                c = c >> 1;
                if (mix) c = c ^ 5'h14;
            end
            crc5_token = ~c;
        end
    endfunction

    task start_token;
        input [7:0] pid_byte;
        input [10:0] token_data;
        reg [4:0] c5;
        begin
            c5 = crc5_token(token_data);
            tx_mem[0] = 8'h80;
            tx_mem[1] = pid_byte;
            tx_mem[2] = {token_data[7:0]};
            tx_mem[3] = {c5, token_data[10:8]};
            tx_done <= 0;
            tx_nbits <= 32;
            tx_bit_index <= 0;
            tx_phase <= 0;
            tx_ones <= 0;
            tx_stuff <= 0;
            tx_eop_count <= 0;
            tx_line_dp <= 1;
            tx_line_dm <= 0;
            tx_state <= TX_BITS;
        end
    endtask

    task start_data_packet;
        input [7:0] pid_byte;
        input [3:0] n;
        reg [15:0] c16;
        integer x;
        begin
            tx_mem[0] = 8'h80;
            tx_mem[1] = pid_byte;
            for (x=0; x<8; x=x+1)
                if (x<n) tx_mem[x+2] = tx_payload[x];
            c16 = crc16_bytes(n);
            tx_mem[n+2] = c16[7:0];
            tx_mem[n+3] = c16[15:8];
            tx_done <= 0;
            tx_nbits <= (n+4) * 8;
            tx_bit_index <= 0;
            tx_phase <= 0;
            tx_ones <= 0;
            tx_stuff <= 0;
            tx_eop_count <= 0;
            tx_line_dp <= 1;
            tx_line_dm <= 0;
            tx_state <= TX_BITS;
        end
    endtask

    task start_handshake;
        input [7:0] pid_byte;
        begin
            tx_mem[0] = 8'h80;
            tx_mem[1] = pid_byte;
            tx_done <= 0;
            tx_nbits <= 16;
            tx_bit_index <= 0;
            tx_phase <= 0;
            tx_ones <= 0;
            tx_stuff <= 0;
            tx_eop_count <= 0;
            tx_line_dp <= 1;
            tx_line_dm <= 0;
            tx_state <= TX_BITS;
        end
    endtask

    function [7:0] pid_full;
        input [3:0] p;
        begin pid_full = {~p,p}; end
    endfunction

    function [15:0] crc16_rx;
        input [3:0] n;
        integer x,y;
        reg [15:0] c;
        reg mix;
        begin
            c=16'hffff;
            for (x=0;x<12;x=x+1) begin
                if (x<n) begin
                    for (y=0;y<8;y=y+1) begin
                        mix=c[0]^rx_mem[x+2][y];
                        c=c>>1;
                        if (mix)c=c^16'ha001;
                    end
                end
            end
            crc16_rx=~c;
        end
    endfunction

    function [4:0] crc5_rx;
        input [10:0] t;
        integer x;
        reg [4:0] c;
        reg mix;
        begin
            c=5'h1f;
            for (x=0;x<11;x=x+1) begin
                mix=c[0]^t[x];
                c=c>>1;
                if(mix)c=c^5'h14;
            end
            crc5_rx=~c;
        end
    endfunction

    assign tx_active = (tx_state != TX_IDLE);
    assign tx_oe = tx_active | ctrl[3] | ctrl[4];
    assign tx_dp = ctrl[3] ? 1'b0 : ctrl[4] ? 1'b0 : tx_line_dp;
    assign tx_dm = ctrl[3] ? 1'b0 : ctrl[4] ? 1'b1 : tx_line_dm;
    assign buf_we = ((host_state == H_RXCOPY) || (dev_state == D_RXCOPY)) &&
                    (rx_copy_index < rx_copy_len);
    assign buf_wdata = rx_payload[rx_copy_index[2:0]];
    assign buf_inc = buf_we || ((host_state == H_COPY) && (copy_index < tx_payload_len)) ||
                     ((dev_state == D_TXCOPY) && (tx_copy_index < tx_copy_len));

    always @(posedge clk) begin
        if (!rst_n) begin
            dp_s <= 2'b11;
            dm_s <= 2'b00;
            dp_sync <= 1'b1;
            dm_sync <= 1'b0;
            rx_phase <= 0;
            se0_count <= 0;
            line_idle_count <= 0;
            line_sample_dp <= 1;
            line_sample_dm <= 0;
            line_reported_dp <= 1;
            line_reported_dm <= 0;
            se0_long <= 0;
            old_se0_long <= 0;
            sof_timer <= 0;
            sof_pending <= 0;
            frame <= 0;
            host_state <= H_IDLE;
            dev_state <= D_IDLE;
            tx_state <= TX_IDLE;
            tx_phase <= 0;
            tx_bit_index <= 0;
            tx_eop_count <= 0;
            tx_ones <= 0;
            tx_stuff <= 0;
            tx_line_dp <= 1;
            tx_line_dm <= 0;
            tx_done <= 0;
            tx_nbits <= 0;
            tx_payload_len <= 0;
            host_op <= 0;
            host_ep <= 0;
            host_data1 <= 0;
            host_is_setup <= 0;
            copy_index <= 0;
            rx_copy_len <= 0;
            rx_copy_index <= 0;
            tx_copy_len <= 0;
            tx_copy_index <= 0;
            timeout_count <= 0;
            rx_payload_len <= 0;
            dev_op <= 0;
            dev_ep <= 0;
            dev_any <= 0;
            dev_data1 <= 0;
            rx_addr <= 0;
            rx_ep <= 0;
            rx_pid <= 0;
            rx_error <= 0;
            rx_byte_count <= 0;
            rx_bit_count <= 0;
            rx_shift <= 0;
            rx_ones <= 0;
            rx_active <= 0;
            rx_last_dp <= 1;
            rx_last_dm <= 0;
            rx_line_dp <= 1;
            rx_line_dm <= 0;
            rx_eop <= 0;
            rx_bad_stuff <= 0;
            rx_eop_se0 <= 0;
            rx_bits_en <= 0;
            rx_packet_done <= 0;
            pending_handshake <= 0;
            pending_op_complete <= 0;
            pending_result_pid <= 0;
            pending_result_err <= 0;
            pending_rx_pid <= 0;
            pending_rx_data1 <= 0;
            pending_valid <= 0;
            busy <= 0;
            bufptr_clr <= 0;
            op_done <= 0;
            res_wr <= 0;
            res_val <= 0;
            tok_wr <= 0;
            tok_val <= 0;
            len_wr <= 0;
            len_val <= 0;
            irq_set <= 0;
        end else begin
            dp_s <= {dp_s[0], dp_in};
            dm_s <= {dm_s[0], dm_in};
            dp_sync <= dp_s[1];
            dm_sync <= dm_s[1];
            tx_done <= 0;
            bufptr_clr <= 0;
            op_done <= 0;
            res_wr <= 0;
            tok_wr <= 0;
            len_wr <= 0;
            irq_set <= 0;
            rx_packet_done <= 0;

            // SE0 detect, independent of transmitter to expose the actual bus state.
            if (dp_s[1] == 0 && dm_s[1] == 0) begin
                if (se0_count != 8'hff)
                    se0_count <= se0_count + 1;
                if (se0_count >= 8'd127)
                    se0_long <= 1;
            end else begin
                se0_count <= 0;
                se0_long <= 0;
            end
            if (se0_long != old_se0_long) begin
                irq_set[2] <= 1;
                old_se0_long <= se0_long;
                if (se0_long) begin
                    line_reported_dp <= 0;
                    line_reported_dm <= 0;
                end
            end
            if (tx_oe || rx_active || (dp_s[1] == 0 && dm_s[1] == 0)) begin
                line_idle_count <= 0;
            end else if (dp_s[1] != line_sample_dp || dm_s[1] != line_sample_dm) begin
                line_sample_dp <= dp_s[1];
                line_sample_dm <= dm_s[1];
                line_idle_count <= 0;
            end else if (line_idle_count != 8'hff) begin
                line_idle_count <= line_idle_count + 1;
            end
            if (line_idle_count == 8'd127 &&
                (dp_s[1] != line_reported_dp || dm_s[1] != line_reported_dm)) begin
                line_reported_dp <= dp_s[1];
                line_reported_dm <= dm_s[1];
                irq_set[2] <= 1;
            end

            // 1 ms host SOF timer and guard window.
            if (ctrl[0] && ctrl[2]) begin
                if (sof_timer == 16'd47999) begin
                    sof_timer <= 0;
                    frame <= frame + 1;
                    sof_pending <= 1;
                end else sof_timer <= sof_timer + 1;
            end else begin
                sof_timer <= 0;
                sof_pending <= 0;
            end

            // Four-times oversampled NRZI receive path. Ignore own transmission.
            if (tx_oe) begin
                rx_active <= 0;
                rx_phase <= 0;
                rx_line_dp <= dp_s[1];
                rx_line_dm <= dm_s[1];
                rx_last_dp <= dp_s[1];
                rx_last_dm <= dm_s[1];
            end else begin
                if (dp_s[1] != rx_line_dp || dm_s[1] != rx_line_dm) begin
                    rx_phase <= 0;
                    if (dp_s[1] != dm_s[1])
                        rx_bits_en <= 1;
                end else
                    rx_phase <= rx_phase + 1;
                rx_line_dp <= dp_s[1];
                rx_line_dm <= dm_s[1];

                if (rx_phase == 1) begin
                    if (dp_s[1] == 0 && dm_s[1] == 0) begin
                        if (rx_active) begin
                            rx_eop <= 1;
                            if (rx_eop_se0 != 7)
                                rx_eop_se0 <= rx_eop_se0 + 1;
                        end
                    end else if (rx_eop) begin
                        if (dp_s[1] && !dm_s[1]) begin
                            rx_active <= 0;
                            rx_eop <= 0;
                            rx_eop_se0 <= 0;
                            rx_packet_done <= 1;
                        end
                    end else if (!rx_active) begin
                        // Start-of-packet begins with J -> K, first SYNC zero.
                        if (rx_last_dp && !rx_last_dm && !dp_s[1] && dm_s[1]) begin
                            rx_active <= 1;
                            rx_byte_count <= 0;
                            rx_bit_count <= 1;
                            rx_shift <= 0;
                            rx_last_dp <= dp_s[1];
                            rx_last_dm <= dm_s[1];
                            rx_ones <= 0;
                            rx_bad_stuff <= 0;
                            rx_bits_en <= 0;
                        end
                    end else if (rx_bits_en) begin
                        // decoded bit is 1 when NRZI line state did not change
                        if (dp_s[1] == rx_last_dp && dm_s[1] == rx_last_dm) begin
                            if (rx_ones == 6) begin
                                rx_ones <= 0; // stuffed zero dropped
                            end else begin
                                rx_shift[rx_bit_count] <= 1'b1;
                                rx_bit_count <= rx_bit_count + 1;
                                rx_ones <= rx_ones + 1;
                                if (rx_bit_count == 7) begin
                                    if (rx_byte_count < 16)
                                        rx_mem[rx_byte_count[3:0]] <= {1'b1,rx_shift[6:0]};
                                    rx_byte_count <= rx_byte_count + 1;
                                    rx_bit_count <= 0;
                                end
                            end
                        end else begin
                            if (rx_ones == 6) begin
                                rx_bad_stuff <= 1;
                                rx_ones <= 0;
                            end else begin
                                rx_shift[rx_bit_count] <= 1'b0;
                                rx_bit_count <= rx_bit_count + 1;
                                rx_ones <= 0;
                                if (rx_bit_count == 7) begin
                                    if (rx_byte_count < 16)
                                        rx_mem[rx_byte_count[3:0]] <= {1'b0,rx_shift[6:0]};
                                    rx_byte_count <= rx_byte_count + 1;
                                    rx_bit_count <= 0;
                                end
                            end
                        end
                    end
                    if (dp_s[1] != 0 || dm_s[1] != 0) begin
                        rx_last_dp <= dp_s[1];
                        rx_last_dm <= dm_s[1];
                    end
                end
            end

            // Receive packet interpretation on EOP. rx_mem[0] is SYNC.
            if (rx_packet_done && rx_byte_count >= 2) begin
                if (rx_mem[1][7:4] == ~rx_mem[1][3:0] && !rx_bad_stuff) begin
                    rx_pid <= rx_mem[1][3:0];
                    if (!ctrl[0]) begin
                        // Device role: addressed token reception
                        if ((rx_mem[1][3:0] == 4'h1 || rx_mem[1][3:0] == 4'h5 ||
                             rx_mem[1][3:0] == 4'h9 || rx_mem[1][3:0] == 4'hd) && rx_byte_count == 4) begin
                            token_tmp = {rx_mem[3][2:0], rx_mem[2]};
                            crc5_tmp = crc5_rx(token_tmp);
                            if (crc5_tmp == rx_mem[3][7:3]) begin
                                if (rx_mem[1][3:0] == 4'h5) begin
                                    frame <= token_tmp;
                                    irq_set[1] <= 1;
                                end else if (token_tmp[10:4] == addr_reg) begin
                                    tok_wr <= 1;
                                    tok_val <= {rx_mem[1][3:0], token_tmp[3:0]};
                                    rx_addr <= token_tmp[10:4];
                                    rx_ep <= token_tmp[3:0];
                                    rx_pid <= rx_mem[1][3:0];
                                    dev_op <= cmd_op;
                                    dev_ep <= cmd_ep;
                                    dev_any <= cmd_any;
                                    dev_data1 <= cmd_data1;
                                    if (rx_mem[1][3:0] == 4'h9) begin
                                        if (cmd_op == 2 && cmd_ep == token_tmp[3:0]) begin
                                            tx_copy_len <= len_reg;
                                            tx_copy_index <= 0;
                                            bufptr_clr <= 1;
                                            dev_state <= D_TXCOPY;
                                        end else begin
                                            start_handshake(pid_full(cmd_op == 3 && cmd_ep == token_tmp[3:0] ? 4'he : 4'ha));
                                            irq_set[3] <= 1;
                                        end
                                    end else if (rx_mem[1][3:0] == 4'h1 || rx_mem[1][3:0] == 4'hd) begin
                                        pending_valid <= 1;
                                        pending_op_complete <= 0;
                                        if (rx_mem[1][3:0] == 4'hd)
                                            pending_handshake <= (cmd_op != 0) ? 4'h2 : 4'ha;
                                        else if ((cmd_op == 1 && (cmd_any || cmd_ep == token_tmp[3:0])))
                                            pending_handshake <= 4'h2;
                                        else if (cmd_op == 3 && cmd_ep == token_tmp[3:0])
                                            pending_handshake <= 4'he;
                                        else pending_handshake <= 4'ha;
                                        if (pending_handshake != 4'h2)
                                            irq_set[3] <= 1;
                                        dev_state <= D_WAITDATA;
                                    end
                                end
                            end
                        end else if ((rx_mem[1][3:0] == 4'h3 || rx_mem[1][3:0] == 4'hb) && dev_state == D_WAITDATA) begin
                            // Device DATA0/DATA1. Only commit if this operation accepts it.
                            crc16_tmp = crc16_rx(rx_byte_count - 4);
                            if (rx_byte_count >= 4 && {rx_mem[rx_byte_count-1],rx_mem[rx_byte_count-2]} == crc16_tmp) begin
                                rx_payload_len <= rx_byte_count - 4;
                                for (i=0;i<8;i=i+1)
                                    if (i < rx_byte_count-4)
                                        rx_payload[i] <= rx_mem[i+2];
                                pending_rx_pid <= (rx_mem[1][3:0] == 4'hb) ? 4'hb : 4'h3;
                                pending_rx_data1 <= (rx_mem[1][3:0] == 4'hb);
                                if (pending_handshake == 4'h2) begin
                                    rx_copy_index <= 0;
                                    rx_copy_len <= rx_byte_count - 4;
                                    bufptr_clr <= 1;
                                    dev_state <= D_RXCOPY;
                                    pending_op_complete <= 1;
                                end else begin
                                    start_handshake(pid_full(pending_handshake));
                                    dev_state <= D_IDLE;
                                    pending_valid <= 0;
                                end
                            end
                        end else if (rx_mem[1][3:0] == 4'h2 && dev_state == D_WAITACK) begin
                            op_done <= 1;
                            res_wr <= 1;
                            res_val <= {1'b0,4'b0,4'h9};
                            irq_set[0] <= 1;
                            dev_state <= D_IDLE;
                        end
                    end else begin
                        // Host receives handshake or data while command waits.
                        if (host_state == H_WAIT && rx_mem[1][3:0] == 4'ha) begin
                            busy <= 0;
                            host_state <= H_IDLE;
                            op_done <= 1;
                            res_wr <= 1;
                            res_val <= {1'b0,4'b0,4'ha};
                            irq_set[0] <= 1;
                        end else if (host_state == H_WAIT && rx_mem[1][3:0] == 4'he) begin
                            busy <= 0;
                            host_state <= H_IDLE;
                            op_done <= 1;
                            res_wr <= 1;
                            res_val <= {1'b0,4'b0,4'he};
                            irq_set[0] <= 1;
                        end else if (host_state == H_WAIT && rx_mem[1][3:0] == 4'h3 ||
                                     host_state == H_WAIT && rx_mem[1][3:0] == 4'hb) begin
                            crc16_tmp = crc16_rx(rx_byte_count - 4);
                            if (rx_byte_count >= 4 && {rx_mem[rx_byte_count-1],rx_mem[rx_byte_count-2]} == crc16_tmp) begin
                                if (rx_byte_count - 4 > 8) begin
                                    busy <= 0;
                                    host_state <= H_IDLE;
                                    op_done <= 1;
                                    res_wr <= 1;
                                    res_val <= {1'b0,3'd3,rx_mem[1][3:0]};
                                    irq_set[0] <= 1;
                                end else begin
                                    rx_payload_len <= rx_byte_count - 4;
                                    for (i=0;i<8;i=i+1)
                                        if (i < rx_byte_count-4) rx_payload[i] <= rx_mem[i+2];
                                    rx_copy_len <= rx_byte_count - 4;
                                    rx_copy_index <= 0;
                                    bufptr_clr <= 1;
                                    host_state <= H_RXCOPY;
                                    pending_rx_pid <= rx_mem[1][3:0];
                                end
                            end else begin
                                busy <= 0;
                                host_state <= H_IDLE;
                                op_done <= 1;
                                res_wr <= 1;
                                res_val <= {1'b0,3'd2,4'h0};
                                irq_set[0] <= 1;
                            end
                        end
                    end
                end
            end

            // Host transmitter/transaction sequencing.
            case (host_state)
                H_IDLE: begin
                    busy <= 0;
                    if (ctrl[0] && sof_pending && tx_state == TX_IDLE) begin
                        token_tmp = frame;
                        start_token(pid_full(4'h5), token_tmp);
                        host_state <= H_SOF;
                    end else if (cmd_wr && ctrl[0] && cmd_op != 3 &&
                                 (!ctrl[2] || sof_timer < 16'd46976) && !sof_pending && tx_state == TX_IDLE) begin
                        busy <= 1;
                        host_op <= cmd_op;
                        host_ep <= cmd_ep;
                        host_data1 <= cmd_data1;
                        host_is_setup <= (cmd_op == 0);
                        copy_index <= 0;
                        tx_payload_len <= (cmd_op == 2) ? 0 : len_reg;
                        bufptr_clr <= 1;
                        if (cmd_op == 2) begin
                            start_token(pid_full(4'h9), {addr_reg,cmd_ep});
                            host_state <= H_TOKEN;
                        end else begin
                            host_state <= H_COPY;
                        end
                    end
                end
                H_COPY: begin
                    if (copy_index < tx_payload_len) begin
                        tx_payload[copy_index[2:0]] <= buf_rdata;
                        copy_index <= copy_index + 1;
                    end else begin
                        start_token(pid_full(host_is_setup ? 4'hd : 4'h1), {addr_reg,host_ep});
                        host_state <= H_TOKEN;
                    end
                end
                H_TOKEN: if (tx_done) begin
                    if (host_op == 2) begin
                        timeout_count <= 0;
                        host_state <= H_WAIT;
                    end else begin
                        start_data_packet(pid_full(host_is_setup || !host_data1 ? 4'h3 : 4'hb), tx_payload_len);
                        host_state <= H_DATA;
                    end
                end
                H_DATA: if (tx_done) begin
                    timeout_count <= 0;
                    host_state <= H_WAIT;
                end
                H_WAIT: begin
                    if (rx_active) begin
                        timeout_count <= 0;
                    end else if (timeout_count == 79) begin
                        busy <= 0;
                        host_state <= H_IDLE;
                        op_done <= 1;
                        res_wr <= 1;
                        res_val <= {1'b0,3'd1,4'h0};
                        irq_set[0] <= 1;
                    end else timeout_count <= timeout_count + 1;
                end
                H_RXCOPY: begin
                    if (rx_copy_index < rx_copy_len) begin
                        rx_copy_index <= rx_copy_index + 1;
                    end else begin
                        len_wr <= 1;
                        len_val <= rx_copy_len;
                        start_handshake(pid_full(4'h2));
                        host_state <= H_ACK;
                    end
                end
                H_ACK: if (tx_done) begin
                    busy <= 0;
                    host_state <= H_IDLE;
                    op_done <= 1;
                    res_wr <= 1;
                    res_val <= {pending_rx_pid == 4'hb,3'b0,pending_rx_pid};
                    irq_set[0] <= 1;
                end
                H_SOF: if (tx_done) begin
                    host_state <= H_IDLE;
                    sof_pending <= 0;
                    irq_set[1] <= 1;
                end
                default: host_state <= H_IDLE;
            endcase

            // Device transmit and receive copy sequencing.
            case (dev_state)
                D_TXCOPY: begin
                    if (tx_copy_index < tx_copy_len) begin
                        tx_payload[tx_copy_index[2:0]] <= buf_rdata;
                        tx_copy_index <= tx_copy_index + 1;
                    end else begin
                        tx_payload_len <= tx_copy_len;
                        start_data_packet(pid_full(dev_data1 ? 4'hb : 4'h3), tx_copy_len);
                        dev_state <= D_WAITACK;
                    end
                end
                D_RXCOPY: begin
                    if (rx_copy_index < rx_copy_len) begin
                        rx_copy_index <= rx_copy_index + 1;
                    end else begin
                        len_wr <= 1;
                        len_val <= rx_copy_len;
                        start_handshake(pid_full(4'h2));
                        if (pending_op_complete) begin
                            op_done <= 1;
                            res_wr <= 1;
                            res_val <= {pending_rx_data1,3'b0,tok_val[7:4]};
                            irq_set[0] <= 1;
                            dev_state <= D_IDLE;
                            pending_valid <= 0;
                        end else dev_state <= D_IDLE;
                    end
                end
                D_WAITACK: begin end
                default: begin end
            endcase

            // SOF send completes independently of host command.

            // Bit-stuffed NRZI packet transmitter.
            case (tx_state)
                TX_IDLE: begin
                    tx_phase <= 0;
                    tx_line_dp <= 1;
                    tx_line_dm <= 0;
                end
                TX_BITS: begin
                    if (tx_phase == 3) begin
                        tx_phase <= 0;
                        if (tx_stuff) begin
                            tx_bit <= 0;
                            tx_stuff <= 0;
                            tx_ones <= 0;
                            tx_line_dp <= ~tx_line_dp;
                            tx_line_dm <= ~tx_line_dm;
                        end else if (tx_bit_index >= tx_nbits) begin
                            tx_state <= TX_EOP0;
                            tx_eop_count <= 0;
                            tx_line_dp <= 0;
                            tx_line_dm <= 0;
                        end else begin
                            tx_raw_bit = tx_mem[tx_bit_index[6:3]];
                            tx_bit <= tx_raw_bit[tx_bit_index[2:0]];
                            tx_bit_index <= tx_bit_index + 1;
                            if (tx_raw_bit[tx_bit_index[2:0]]) begin
                                tx_ones <= tx_ones + 1;
                                if (tx_ones == 5) tx_stuff <= 1;
                            end else begin
                                tx_ones <= 0;
                                tx_line_dp <= ~tx_line_dp;
                                tx_line_dm <= ~tx_line_dm;
                            end
                        end
                    end else tx_phase <= tx_phase + 1;
                end
                TX_EOP0: begin
                    tx_line_dp <= 0;
                    tx_line_dm <= 0;
                    if (tx_phase == 3) begin
                        tx_phase <= 0;
                        if (tx_eop_count == 1) begin
                            tx_state <= TX_EOPJ;
                            tx_eop_count <= 0;
                            tx_line_dp <= 1;
                            tx_line_dm <= 0;
                        end else tx_eop_count <= tx_eop_count + 1;
                    end else tx_phase <= tx_phase + 1;
                end
                TX_EOPJ: begin
                    tx_line_dp <= 1;
                    tx_line_dm <= 0;
                    if (tx_phase == 3) begin
                        tx_state <= TX_IDLE;
                        tx_phase <= 0;
                        tx_done <= 1;
                    end else tx_phase <= tx_phase + 1;
                end
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    wire _unused = &{dp_in, dm_in, rx_ep, rx_addr, rx_error, pending_valid,
                     pending_result_pid, pending_result_err, pending_handshake,
                     1'b0};

endmodule
