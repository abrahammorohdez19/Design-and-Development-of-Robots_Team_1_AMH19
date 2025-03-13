%% TEST 2 INVERSE KINEMATICS CHALLENGE AMH19-CMH156
%-------------------PRELIMINAR FINAL CODE-------------------

clear; close; clc
% Configuración del socket UDP
udp_receiver = udpport("datagram", "LocalPort", 5005);
disp("🛰 Esperando señales de Python...");

contador_1s = 0;
simulacion_activada = false;

while true
    if udp_receiver.NumDatagramsAvailable > 0
        datagrama = read(udp_receiver, 1, "uint8");
        signal = char(datagrama.Data);

        fprintf("📥 Señal recibida: %s\n", signal);

        if ~simulacion_activada
            % Lógica del contador
            if signal == '1'
                contador_1s = contador_1s + 1;
                fprintf("✅ '1' detectado (%d consecutivos)\n", contador_1s);
            else
                contador_1s = 0;
            end

            % Activar simulación cuando haya 3 consecutivos
            if contador_1s == 3
                simulacion_activada = true; % bandera para no depender de señales futuras
                disp("🚀 3 señales '1' consecutivas recibidas. Ejecutando simulación...");

                % -------- Código de trayectoria --------
                t_max = 15; % Ahora la simulación dura 15 segundos
                t = linspace(0, t_max, 150)';

                q_inicial = [0, 0, 0, 0, 0];
                q_bajar = [0, -0.7070, -0.7404, 0, -0.6620];
                q_subir = [0, -1.1967, -0.3030, 0, -1.4997];
                q_comer = [pi/4, 0, 0, 0, 0]; % Nueva posición
                q_a_boca = [0, 0, 0, 0, 0];
   
                % Definir tiempos para cada fase
                T1 = linspace(0, 3, 30)';  % Inicial → Bajar
                T2 = linspace(3, 4, 10)';  % Mantenerse abajo
                T3 = linspace(4, 6, 20)';  % Bajar → Subir
                T4 = linspace(6, 8, 20)';  % Subir → Comer
                T5 = linspace(8, 11, 30)'; % *** Pausa en Comer (3 seg) ***
                T6 = linspace(11, 13, 20)'; % Comer → Boca
                T7 = linspace(13, 15, 20)'; % Boca → Final

                Q = cell(1, 5);
                for j = 1:5
                    Q{j} = [T1, linspace(q_inicial(j), q_bajar(j), length(T1))';
                            T2, repmat(q_bajar(j), length(T2), 1);
                            T3, linspace(q_bajar(j), q_subir(j), length(T3))';
                            T4, linspace(q_subir(j), q_comer(j), length(T4))';
                            T5, repmat(q_comer(j), length(T5), 1); % *** PAUSA EN q_comer ***
                            T6, linspace(q_comer(j), q_a_boca(j), length(T6))';
                            T7, repmat(q_a_boca(j), length(T7), 1)];
                end

                Q1 = Q{1}; Q2 = Q{2}; Q3 = Q{3}; Q4 = Q{4}; Q5 = Q{5};

                sim('AMH19_Mark1_Complete_URDF.slx');
                disp("🤖 Simulación completada.");
            end
        else
            fprintf("📡 Señal recibida (post-simulación): %s\n", signal);
        end
    end

    pause(0.4); % AUMENTA el tiempo entre iteraciones para suavidad
end
