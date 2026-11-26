
/*
 * Supervisor CNC V4 - Tiny Tapeout (GF)
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// ============================================================
// Wrapper de Tiny Tapeout (modulo top)
// ============================================================
// ui_in[0] = step_x
// ui_in[1] = step_y
// ui_in[2] = vigilar_x
// ui_in[3] = vigilar_y
//
// uo_out[0] = falla_x
// uo_out[1] = falla_y
// uo_out[2] = falla_general
// uo_out[3] = salida_segura
// ============================================================

module tt_um_supervisor_cnc_v4 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    wire falla_x;
    wire falla_y;
    wire falla_general;
    wire salida_segura;

    supervisor_cnc_v4 #(
        .PRESCALE(14'd10000),
        .TIMEOUT_TICKS(11'd2000)
    ) u_supervisor (
        .clk           (clk),
        .rst           (~rst_n),
        .step_x        (ui_in[0]),
        .step_y        (ui_in[1]),
        .vigilar_x     (ui_in[2]),
        .vigilar_y     (ui_in[3]),
        .falla_x       (falla_x),
        .falla_y       (falla_y),
        .falla_general (falla_general),
        .salida_segura (salida_segura)
    );

    assign uo_out = {
        4'b0000,
        salida_segura,
        falla_general,
        falla_y,
        falla_x
    };

    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    // Evita advertencias por entradas sin utilizar
    wire _unused = &{ena, ui_in[7:4], uio_in, 1'b0};

endmodule


// ============================================================
// Supervisor CNC V4
// Frecuencia de reloj: 10 MHz
// Divisor: 10000 ciclos = 1 ms
// Tiempo nominal de vigilancia: 2000 ticks = 2 segundos
// ============================================================

module supervisor_cnc_v4 #(
    parameter [13:0] PRESCALE = 14'd10000,
    parameter [10:0] TIMEOUT_TICKS = 11'd2000
)(
    input  wire clk,
    input  wire rst,
    input  wire step_x,
    input  wire step_y,
    input  wire vigilar_x,
    input  wire vigilar_y,
    output wire falla_x,
    output wire falla_y,
    output wire falla_general,
    output wire salida_segura
);

    // Constantes con ancho de bits definido
    localparam [13:0] PRESCALE_MAX =
        PRESCALE - 14'd1;

    localparam [10:0] TIMEOUT_MAX =
        TIMEOUT_TICKS - 11'd1;

    // Divisor de frecuencia
    reg [13:0] divisor;

    wire tick_ms = (divisor == PRESCALE_MAX);

    // Contadores de vigilancia
    reg [10:0] tiempo_x;
    reg [10:0] tiempo_y;

    // Registros de falla
    reg fx, fy;

    // Sincronizacion de entradas asincronas
    reg x_meta, x_sync, x_prev;
    reg y_meta, y_sync, y_prev;
    reg vx_meta, vx_sync;
    reg vy_meta, vy_sync;

    // Deteccion de flancos ascendentes
    wire pulso_x = x_sync & ~x_prev;
    wire pulso_y = y_sync & ~y_prev;

    always @(posedge clk or posedge rst) begin

        if (rst) begin

            divisor  <= 14'd0;
            tiempo_x <= 11'd0;
            tiempo_y <= 11'd0;

            fx <= 1'b0;
            fy <= 1'b0;

            x_meta <= 1'b0;
            x_sync <= 1'b0;
            x_prev <= 1'b0;

            y_meta <= 1'b0;
            y_sync <= 1'b0;
            y_prev <= 1'b0;

            vx_meta <= 1'b0;
            vx_sync <= 1'b0;

            vy_meta <= 1'b0;
            vy_sync <= 1'b0;

        end else begin

            // Sincronizacion del eje X
            x_meta <= step_x;
            x_sync <= x_meta;
            x_prev <= x_sync;

            // Sincronizacion del eje Y
            y_meta <= step_y;
            y_sync <= y_meta;
            y_prev <= y_sync;

            // Sincronizacion de habilitaciones
            vx_meta <= vigilar_x;
            vx_sync <= vx_meta;

            vy_meta <= vigilar_y;
            vy_sync <= vy_meta;

            // Divisor de frecuencia
            if (tick_ms)
                divisor <= 14'd0;
            else
                divisor <= divisor + 14'd1;

            // ========================================
            // Vigilancia del eje X
            // ========================================

            if (!fx) begin

                if (!vx_sync || pulso_x)
                    tiempo_x <= 11'd0;

                else if (tick_ms) begin

                    if (tiempo_x == TIMEOUT_MAX)
                        fx <= 1'b1;
                    else
                        tiempo_x <= tiempo_x + 11'd1;

                end
            end

            // ========================================
            // Vigilancia del eje Y
            // ========================================

            if (!fy) begin

                if (!vy_sync || pulso_y)
                    tiempo_y <= 11'd0;

                else if (tick_ms) begin

                    if (tiempo_y == TIMEOUT_MAX)
                        fy <= 1'b1;
                    else
                        tiempo_y <= tiempo_y + 11'd1;

                end
            end
        end
    end

    // Salidas del supervisor
    assign falla_x = fx;
    assign falla_y = fy;
    assign falla_general = fx | fy;

    // 1 = Sin falla detectada
    // 0 = Alarma
    // Solamente indicacion logica
    assign salida_segura = ~(fx | fy);

endmodule

