%% VIQ - Radar Doppler: Versión Final Corregida
clear; clc; close all;

%% Parámetros
port = "COM12"; baud = 921600; fs = 9000; samples = 256;
Vref = 3.3; bitsADC = 12;
FRAME_HEADER = uint8([170 85 170 85]);
bytesData = samples * 2;

%% Calibración
f_transmision = 24e9; c = 3e8;
factor_calibracion = 1.038; 

%% Interfaz
fig = figure('Color', 'w', 'Name', 'Radar Doppler Calibrado', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);
subplot(2,1,1);
max_puntos = 100;
historial_v = zeros(1, max_puntos);
h_plot = plot(historial_v, 'LineWidth', 2, 'Color', [0 0.44 0.74]);
grid on; ylabel('km/h'); title('Perfil de Velocidad Instantánea'); ylim([0 100]);

subplot(2,1,2); axis off;
txtVel = text(0.5, 0.6, 'LISTO', 'FontSize', 90, 'FontWeight', 'bold', 'Color', [0.5 0.5 0.5], 'HorizontalAlignment', 'center');
txtStatus = text(0.5, 0.2, 'ESPERANDO...', 'FontSize', 20, 'HorizontalAlignment', 'center');

%% Lógica de Ventana y Filtro de Picos
estado = 0; acumulador_v = [];
umbral_on = 0.022; umbral_off = 0.015;
frames_gracia = 5; contador_vacio = 0;

s = serialport(port, baud); flush(s); acc = uint8([]);

while ishandle(fig)
    if s.NumBytesAvailable > 0
        acc = [acc; read(s, s.NumBytesAvailable, "uint8")'];
    end
    
    if numel(acc) >= 4 + bytesData
        idx = strfind(acc', FRAME_HEADER);
        if ~isempty(idx)
            p = idx(1);
            if p + 4 + bytesData - 1 <= numel(acc)
                data = acc(p+4 : p+3+bytesData); acc(1:p+3+bytesData) = [];
                
                % Reconstrucción ADC
                lsb = double(data(1:2:end)); msb = double(data(2:2:end));
                adc = bitor(lsb, bitshift(bitand(msb,15),8));
                y = adc * (Vref / (2^bitsADC - 1));
                
                % Cálculo de Velocidad
                y_ac = y - mean(y);
                f_doppler = (sum(abs(diff(y_ac > 0))) * fs) / (2 * samples);
                v_kmh = ((f_doppler * c) / (2 * f_transmision) * factor_calibracion) * 3.6;
                
                % --- ACTUALIZACIÓN DE GRÁFICA (Línea Corregida) ---
                v_data = get(h_plot, 'YData');
                set(h_plot, 'YData', [v_data(2:end), v_kmh]);
                
                % --- LÓGICA DE VENTANA ---
                intensidad = std(y);
                if estado == 0
                    if intensidad > umbral_on
                        estado = 1; acumulador_v = []; contador_vacio = 0;
                        set(txtStatus, 'String', 'DETECTANDO...', 'Color', 'r');
                    end
                else
                    if intensidad > umbral_off
                        % Solo acumulamos si la velocidad no es ruido (pico real)
                        if v_kmh > 10
                            acumulador_v = [acumulador_v, v_kmh];
                        end
                        contador_vacio = 0;
                    else
                        contador_vacio = contador_vacio + 1;
                        if contador_vacio >= frames_gracia
                            estado = 0;
                            if ~isempty(acumulador_v)
                                % Filtro de Picos Altos (Top 40%)
                                sorted_v = sort(acumulador_v, 'descend');
                                n_top = max(1, round(length(sorted_v) * 0.4)); 
                                v_final = mean(sorted_v(1:n_top));
                                
                                set(txtVel, 'String', sprintf('%.1f km/h', v_final), 'Color', [0 0.5 0]);
                                set(txtStatus, 'String', 'AUTO PROCESADO', 'Color', [0 0 0]);
                            end
                        end
                    end
                end
                drawnow limitrate;
            end
        end
    end
end
clear s;