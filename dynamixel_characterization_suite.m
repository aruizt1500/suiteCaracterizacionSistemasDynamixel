%--------------------------------------------------------------------------
% SUITE DE CARACTERIZACIÓN DE SISTEMAS PARA SERVOMOTOR DYNAMIXEL (v4)
% Fecha: 26 de septiembre de 2025
%
% Descripción:
% Este script proporciona un menú interactivo para realizar diversas
% pruebas de identificación de sistemas en un servomotor Dynamixel,
% permitiendo al usuario elegir entre diferentes metodologías para
% caracterizar la dinámica del actuador.
%--------------------------------------------------------------------------

function dynamixel_characterization_suite()

    % --- CONFIGURACIÓN GLOBAL ---
    % Asegúrate de que estos parámetros sean correctos para tu configuración
    config.PROTOCOL_VERSION = 2.0;
    config.DXL_ID           = 1;
    config.BAUDRATE         = 57600;
    config.DEVICENAME       = 'COM5';
    config.ADDR_TORQUE_ENABLE    = 64;
    config.ADDR_GOAL_POSITION    = 116;
    config.ADDR_PRESENT_POSITION = 132;
    config.ADDR_OPERATING_MODE   = 11;
    config.TICKS_PER_REV    = 4096;
    
    % Cargar la librería del SDK
    lib_name = load_dynamixel_library();
    
    % Inicializar la comunicación con el motor
    port_num = initialize_dynamixel(config, lib_name);
    
    % Bloque principal TRY-CATCH para garantizar el cierre seguro del puerto
    try
        % Bucle del menú principal
        while true
            clc; % Limpiar la consola
            fprintf('=====================================================\n');
            fprintf('  SUITE DE CARACTERIZACIÓN DE SERVOMOTOR DYNAMIXEL\n');
            fprintf('=====================================================\n');
            fprintf('1. Modelo de 1er Orden (Respuesta al Escalón Simple)\n');
            fprintf('2. Modelo de 1er Orden Robusto (Promediado de Pruebas)\n');
            fprintf('3. Análisis de Linealidad del Sistema\n');
            fprintf('4. Modelo de 2do Orden (Análisis de Sobreimpulso)\n');
            fprintf('5. Identificación Avanzada con Señal PRBS\n');
            fprintf('0. Salir\n');
            fprintf('-----------------------------------------------------\n');
            
            choice = input('Seleccione una opción: ');
            
            % Ejecutar la opción seleccionada
            switch choice
                case 1
                    run_simple_step_response(port_num, config, 1);
                case 2
                    run_robust_step_response(port_num, config);
                case 3
                    run_linearity_analysis(port_num, config);
                case 4
                    run_simple_step_response(port_num, config, 2);
                case 5
                    run_prbs_identification(port_num, config);
                case 0
                    break; % Salir del bucle while
                otherwise
                    fprintf('Opción no válida. Presione Enter para continuar.\n');
                    pause;
            end
            
            if choice ~= 0
                fprintf('\nPrueba finalizada. Presione Enter para volver al menú principal.\n');
                pause;
            end
        end

    catch e
        fprintf(2, 'Ha ocurrido un error: %s\n', e.message);
        fprintf(2, 'En el archivo %s, línea %d\n', e.stack(1).name, e.stack(1).line);
    end
    
    % --- FINALIZACIÓN SEGURA ---
    terminate_dynamixel(port_num, config, lib_name);
    fprintf('Suite de caracterización finalizada.\n');
end

% -------------------------------------------------------------------------
% --- FUNCIONES DE LOS MÓDULOS DE ANÁLISIS ---
% -------------------------------------------------------------------------

function run_simple_step_response(port_num, config, model_order)
    fprintf('\n--- Módulo: Modelo de %do Orden (Escalón Simple) ---\n', model_order);
    pos_inicial_g = input('Introduce la posición INICIAL en grados (0-360): ');
    pos_final_g = input('Introduce la posición FINAL en grados (0-360): ');

    [tiempo, respuesta_g] = execute_step_experiment(port_num, config, pos_inicial_g, pos_final_g);
    
    entrada_g = [zeros(floor(0.5/0.01), 1); ones(length(respuesta_g) - floor(0.5/0.01), 1) * (pos_final_g - pos_inicial_g)];
    datos_exp = iddata(respuesta_g, entrada_g, 0.01);
    
    fprintf('Estimando modelo de %do orden...\n', model_order);
    sys_estimado = tfest(datos_exp, model_order, 0);
    
    disp(sys_estimado);
    analyze_and_display_model(sys_estimado);
    
    figure;
    compare(datos_exp, sys_estimado);
    title(sprintf('Respuesta al Escalón vs. Modelo de %do Orden', model_order));
    legend('Respuesta Real', 'Respuesta del Modelo');
end

function run_robust_step_response(port_num, config)
    fprintf('\n--- Módulo: Modelo de 1er Orden Robusto (Promediado) ---\n');
    pos_inicial_g = input('Introduce la posición INICIAL en grados (0-360): ');
    pos_final_g = input('Introduce la posición FINAL en grados (0-360): ');
    n_runs = input('Introduce el número de experimentos a promediar (ej. 5): ');
    
    all_responses = [];
    fprintf('Ejecutando %d experimentos...\n', n_runs);
    for i = 1:n_runs
        fprintf('  - Ejecución %d de %d...\n', i, n_runs);
        [~, respuesta_g] = execute_step_experiment(port_num, config, pos_inicial_g, pos_final_g);
        all_responses(:, i) = respuesta_g;
    end
    
    respuesta_promedio_g = mean(all_responses, 2);
    fprintf('Experimentos finalizados. Analizando respuesta promediada.\n');
    
    entrada_g = [zeros(floor(0.5/0.01), 1); ones(length(respuesta_promedio_g) - floor(0.5/0.01), 1) * (pos_final_g - pos_inicial_g)];
    datos_exp = iddata(respuesta_promedio_g, entrada_g, 0.01);
    
    sys_estimado = tfest(datos_exp, 1, 0);
    disp(sys_estimado);
    analyze_and_display_model(sys_estimado);
    
    figure;
    compare(datos_exp, sys_estimado);
    title('Respuesta Promediada vs. Modelo de 1er Orden Robusto');
end

function run_linearity_analysis(port_num, config)
    fprintf('\n--- Módulo: Análisis de Linealidad ---\n');
    fprintf('Se realizarán 3 pruebas con escalones de diferente amplitud.\n');
    
    amplitudes = {'Pequeño', 'Mediano', 'Grande'};
    results = {};
    
    for i = 1:3
        fprintf('\n--- Configurando Escalón %s ---\n', amplitudes{i});
        pos_inicial_g = input(sprintf('Posición INICIAL para escalón %s (grados): ', amplitudes{i}));
        pos_final_g = input(sprintf('Posición FINAL para escalón %s (grados): ', amplitudes{i}));
        
        [~, respuesta_g] = execute_step_experiment(port_num, config, pos_inicial_g, pos_final_g);
        entrada_g = [zeros(floor(0.5/0.01), 1); ones(length(respuesta_g) - floor(0.5/0.01), 1) * (pos_final_g - pos_inicial_g)];
        datos_exp = iddata(respuesta_g, entrada_g, 0.01);
        
        sys = tfest(datos_exp, 1, 0);
        [K, T] = get_first_order_params(sys);
        results{i} = {amplitudes{i}, K, T};
    end
    
    fprintf('\n--- Resultados del Análisis de Linealidad ---\n');
    fprintf('Amplitud\t| Ganancia (K)\t| Constante de Tiempo (T)\n');
    fprintf('-----------------------------------------------------\n');
    for i = 1:3
        fprintf('%s\t\t| %.4f\t\t| %.4f s\n', results{i}{1}, results{i}{2}, results{i}{3});
    end
    fprintf('\nSi los valores de K y T son similares, el sistema es lineal en este rango.\n');
end

function run_prbs_identification(port_num, config)
    fprintf('\n--- Módulo: Identificación Avanzada con Señal PRBS ---\n');
    fprintf('Esta señal excita múltiples frecuencias para un mejor modelo.\n');
    pos_low_g = input('Introduce el nivel BAJO de posición (grados): ');
    pos_high_g = input('Introduce el nivel ALTO de posición (grados): ');
    duracion_s = 15; % Duración total del experimento PRBS
    Ts = 0.01;
    num_puntos = duracion_s / Ts;

    % Generar señal PRBS
    input_signal_g = idinput(num_puntos, 'prbs', [0 1/(5*Ts)], [pos_low_g pos_high_g]);
    
    fprintf('Moviendo a la posición inicial (%.1f grados)...\n', input_signal_g(1));
    write4ByteTxRx(port_num, config.PROTOCOL_VERSION, config.DXL_ID, config.ADDR_GOAL_POSITION, round(input_signal_g(1) * config.TICKS_PER_REV/360));
    pause(2);
    
    fprintf('Iniciando experimento PRBS...\n');
    output_signal_ticks = zeros(num_puntos, 1);
    tic;
    for i = 1:num_puntos
        pos_goal_ticks = round(input_signal_g(i) * config.TICKS_PER_REV/360);
        write4ByteTxRx(port_num, config.PROTOCOL_VERSION, config.DXL_ID, config.ADDR_GOAL_POSITION, pos_goal_ticks);
        
        % Pausa para mantener el tiempo de muestreo
        while(toc < i*Ts), end
        
        output_signal_ticks(i) = read4ByteTxRx(port_num, config.PROTOCOL_VERSION, config.DXL_ID, config.ADDR_PRESENT_POSITION);
    end
    
    output_signal_g = output_signal_ticks * (360/config.TICKS_PER_REV);
    datos_exp = iddata(output_signal_g, input_signal_g, Ts);
    
    np = input('Introduce el orden del modelo a estimar (ej. 2 para 2do orden): ');
    sys_estimado = tfest(datos_exp, np, 0);
    
    disp(sys_estimado);
    analyze_and_display_model(sys_estimado);
    
    figure;
    compare(datos_exp, sys_estimado);
    title(sprintf('Respuesta a PRBS vs. Modelo de %do Orden', np));
end

% -------------------------------------------------------------------------
% --- FUNCIONES AUXILIARES Y DE HARDWARE ---
% -------------------------------------------------------------------------

function [tiempo, respuesta_g] = execute_step_experiment(port_num, config, pos_inicial_g, pos_final_g)
    % Convierte grados a ticks
    pos_inicial_t = round(pos_inicial_g * config.TICKS_PER_REV/360);
    pos_final_t   = round(pos_final_g * config.TICKS_PER_REV/360);
    
    fprintf('Moviendo a la posición inicial (%.1f grados)...\n', pos_inicial_g);
    write4ByteTxRx(port_num, config.PROTOCOL_VERSION, config.DXL_ID, config.ADDR_GOAL_POSITION, pos_inicial_t);
    pause(2);
    
    % Configuración de la captura
    duracion_pre = 0.5; duracion_post = 2.0; Ts = 0.01;
    n_pre = floor(duracion_pre/Ts); n_post = floor(duracion_post/Ts); n_total = n_pre + n_post;
    
    tiempo = zeros(n_total, 1);
    pos_real_t = zeros(n_total, 1);
    
    fprintf('Iniciando captura...\n');
    tic;
    
    for i=1:n_pre
        pos_real_t(i) = read4ByteTxRx(port_num, config.PROTOCOL_VERSION, config.DXL_ID, config.ADDR_PRESENT_POSITION);
        tiempo(i) = toc;
        while(toc < i*Ts), end
    end
    
    write4ByteTxRx(port_num, config.PROTOCOL_VERSION, config.DXL_ID, config.ADDR_GOAL_POSITION, pos_final_t);
    t_start_post = toc;

    for i=1:n_post
        idx = i + n_pre;
        pos_real_t(idx) = read4ByteTxRx(port_num, config.PROTOCOL_VERSION, config.DXL_ID, config.ADDR_PRESENT_POSITION);
        tiempo(idx) = toc;
        while(toc < t_start_post + i*Ts), end
    end
    
    pos_real_g = pos_real_t * (360/config.TICKS_PER_REV);
    respuesta_g = pos_real_g - pos_inicial_g;
    fprintf('Captura finalizada.\n');
end

function analyze_and_display_model(sys)
    order = length(sys.Denominator) - 1;
    fprintf('\n--- Análisis del Modelo ---\n');
    if order == 1
        [K, T] = get_first_order_params(sys);
        fprintf('Modelo de 1er Orden: G(s) = K / (T*s + 1)\n');
        fprintf('  -> Ganancia Estática (K): %.4f\n', K);
        fprintf('  -> Constante de Tiempo (T): %.4f segundos\n', T);
    elseif order == 2
        den = sys.Denominator;
        num = sys.Numerator;
        wn = sqrt(den(3));
        zeta = den(2) / (2*wn);
        K = num / den(3);
        fprintf('Modelo de 2do Orden: G(s) = K*wn^2 / (s^2 + 2*zeta*wn*s + wn^2)\n');
        fprintf('  -> Ganancia Estática (K): %.4f\n', K);
        fprintf('  -> Frecuencia Natural (wn): %.4f rad/s\n', wn);
        fprintf('  -> Factor de Amortiguamiento (zeta): %.4f\n', zeta);
        if zeta < 1
            fprintf('     (Sistema Subamortiguado: esperaría sobreimpulso)\n');
        elseif zeta == 1
            fprintf('     (Sistema Críticamente Amortiguado)\n');
        else
            fprintf('     (Sistema Sobreamortiguado)\n');
        end
    else
        fprintf('Análisis para modelos de orden %d no implementado.\n', order);
    end
end

function [K, T] = get_first_order_params(sys)
    num = sys.Numerator; den = sys.Denominator;
    K = num / den(2);
    T = 1 / den(2);
end

function lib_name = load_dynamixel_library()
    if strcmp(computer, 'PCWIN64'), lib_name = 'dxl_x64_c';
    elseif strcmp(computer, 'PCWIN'), lib_name = 'dxl_x86_c';
    else, error('Sistema operativo no compatible.');
    end
    if ~libisloaded(lib_name)
        loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
    end
end

function port_num = initialize_dynamixel(config, lib_name)
    port_num = portHandler(config.DEVICENAME);
    packetHandler();
    if ~openPort(port_num), unloadlibrary(lib_name); error('No se pudo abrir el puerto COM.'); end
    if ~setBaudRate(port_num, config.BAUDRATE), unloadlibrary(lib_name); error('No se pudo fijar el Baudrate.'); end
    
    write1ByteTxRx(port_num, config.PROTOCOL_VERSION, config.DXL_ID, config.ADDR_OPERATING_MODE, 3);
    write1ByteTxRx(port_num, config.PROTOCOL_VERSION, config.DXL_ID, config.ADDR_TORQUE_ENABLE, 1);
    fprintf('Dynamixel inicializado y con torque habilitado.\n');
    pause(1); % Pequeña pausa para estabilizar
end

function terminate_dynamixel(port_num, config, lib_name)
    fprintf('\nDeshabilitando torque y cerrando puerto...\n');
    write1ByteTxRx(port_num, config.PROTOCOL_VERSION, config.DXL_ID, config.ADDR_TORQUE_ENABLE, 0);
    closePort(port_num);
    unloadlibrary(lib_name);
end