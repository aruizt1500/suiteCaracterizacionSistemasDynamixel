%--------------------------------------------------------------------------
% SCRIPT INTERACTIVO PARA CARACTERIZACIÓN DE SERVOMOTOR DYNAMIXEL (v3)
% Pide al usuario la posición inicial y final en grados.
%--------------------------------------------------------------------------

% Cargar la librería del SDK de Dynamixel
lib_name = '';
if strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c'; % Para Windows de 64 bits
elseif strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c'; % Para Windows de 32 bits
else
    error('Sistema operativo no compatible. Adapte el nombre de la librería.');
end

if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%==========================================================================
% 1. CONFIGURACIÓN DE LA COMUNICACIÓN
%==========================================================================
PROTOCOL_VERSION    = 2.0;
DXL_ID              = 1;       % <-- AJUSTAR: ID de tu servomotor
BAUDRATE            = 57600;   % <-- AJUSTAR: Baudrate de tu servomotor
DEVICENAME          = 'COM5';  % <-- CONFIRMADO: Puerto COM

% Direcciones del Control Table (verificar para tu modelo de motor)
ADDR_TORQUE_ENABLE    = 64;
ADDR_GOAL_POSITION    = 116;
ADDR_PRESENT_POSITION = 132;
ADDR_OPERATING_MODE   = 11;

% Constantes
TORQUE_ENABLE       = 1;
TORQUE_DISABLE      = 0;
POSITION_MODE       = 3;

% Inicializar y abrir puerto
port_num = portHandler(DEVICENAME);
packetHandler();

if (openPort(port_num))
    fprintf('Puerto COM5 abierto exitosamente.\n');
else
    unloadlibrary(lib_name);
    error('ERROR: No se pudo abrir el puerto COM5.');
end

if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate configurado a %d bps.\n', BAUDRATE);
else
    unloadlibrary(lib_name);
    error('ERROR: No se pudo configurar el baudrate.');
end

%==========================================================================
% 2. CONFIGURACIÓN DEL SERVOMOTOR
%==========================================================================
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, POSITION_MODE);
fprintf('Modo de operación configurado en "Control de Posición".\n');
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
fprintf('Torque habilitado. ¡El motor opondrá resistencia!\n');

%==========================================================================
% 3. EJECUCIÓN DEL EXPERIMENTO (CON ENTRADA DEL USUARIO)
%==========================================================================
% --- ¡NUEVO! CÓDIGO INTERACTIVO ---
fprintf('\n--- CONFIGURACIÓN DEL EXPERIMENTO ---\n');
pos_inicial_grados = input('Introduce la posición INICIAL en grados (0 a 360): ');
pos_final_grados = input('Introduce la posición FINAL en grados (0 a 360): ');

% Validación simple de las entradas
if pos_inicial_grados < 0 || pos_inicial_grados > 360 || pos_final_grados < 0 || pos_final_grados > 360
    error('Las posiciones deben estar en el rango de 0 a 360 grados.');
end
fprintf('Experimento configurado: Movimiento de %.1f a %.1f grados.\n\n', pos_inicial_grados, pos_final_grados);

% Factor de conversión
ticks_a_grados = 360.0 / 4096.0;

% Convertimos a ticks para enviar los comandos al motor
pos_inicial_ticks = round(pos_inicial_grados / ticks_a_grados);
pos_final_ticks   = round(pos_final_grados / ticks_a_grados);

% Mover a la posición inicial y esperar a que se estabilice
fprintf('Moviendo a la posición inicial (%.1f grados)...\n', pos_inicial_grados);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, pos_inicial_ticks);
pause(2);

% --- CAPTURA DE DATOS MEJORADA (EVITA EL WARNING) ---
fprintf('Iniciando captura de datos...\n');
duracion_pre_escalon = 0.5; % [s]
duracion_respuesta   = 2.0; % [s]
Ts_deseado           = 0.01;% [s]

num_muestras_pre  = floor(duracion_pre_escalon / Ts_deseado);
num_muestras_post = floor(duracion_respuesta / Ts_deseado);
num_muestras_total = num_muestras_pre + num_muestras_post;

% Vectores para almacenar datos
tiempo        = zeros(num_muestras_total, 1);
posicion_real_ticks = zeros(num_muestras_total, 1);
entrada_ideal_grados = [zeros(num_muestras_pre, 1); ones(num_muestras_post, 1) * (pos_final_grados - pos_inicial_grados)];

tic; % Iniciar cronómetro

% 1. Captura PRE-ESCALÓN
for i = 1:num_muestras_pre
    posicion_real_ticks(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    tiempo(i) = toc;
    while(toc < i * Ts_deseado), end
end

% 2. APLICACIÓN DEL ESCALÓN
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, pos_final_ticks);

% 3. Captura POST-ESCALÓN
tiempo_inicio_post = toc;
for i = 1:num_muestras_post
    idx = i + num_muestras_pre;
    posicion_real_ticks(idx) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    tiempo(idx) = toc;
    while(toc < tiempo_inicio_post + i * Ts_deseado), end
end

fprintf('Captura de datos finalizada. Se tomaron %d muestras.\n', num_muestras_total);

%==========================================================================
% 4. ANÁLISIS DE DATOS (EN GRADOS)
%==========================================================================
% Convertimos los datos de ticks a grados ANTES del análisis
posicion_real_grados = posicion_real_ticks * ticks_a_grados;

% La respuesta del sistema se calcula como el cambio desde la posición inicial.
% Para la parte pre-escalón, la respuesta es cercana a cero.
respuesta_sistema_grados = posicion_real_grados - pos_inicial_grados;

% Crear el objeto 'iddata'
datos_exp = iddata(respuesta_sistema_grados, entrada_ideal_grados, Ts_deseado);
datos_exp.InputName = 'Escalón de Posición (grados)';
datos_exp.OutputName = 'Respuesta del Motor (grados)';

% Estimar la función de transferencia
fprintf('Estimando la función de transferencia con "tfest"...\n');
sys_estimado = tfest(datos_exp, 1, 0);

disp('----------------------------------------------------');
disp('Función de Transferencia Estimada G(s):');
disp(sys_estimado);

% Extraer y calcular K y T
num_coeffs = sys_estimado.Numerator;
den_coeffs = sys_estimado.Denominator;
K = num_coeffs / den_coeffs(2);
T = 1 / den_coeffs(2);

fprintf('\nParámetros del Modelo de Primer Orden G(s) = K / (T*s + 1):\n');
fprintf('  -> Ganancia Estática (K): %.4f\n', K);
fprintf('  -> Constante de Tiempo (T): %.4f segundos\n', T);
disp('----------------------------------------------------');

%==========================================================================
% 5. VISUALIZACIÓN Y FINALIZACIÓN
%==========================================================================
figure;
compare(datos_exp, sys_estimado);
title('Comparación: Respuesta Experimental vs. Modelo de 1er Orden');
legend('Respuesta Experimental Real', 'Respuesta del Modelo Matemático');
xlabel('Tiempo (s)');
ylabel('Posición (grados desde el inicio)');
grid on;
set(gcf, 'color', 'w');

% Finalización segura
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
closePort(port_num);
unloadlibrary(lib_name);
fprintf('\nComunicación finalizada y torque deshabilitado. ¡Proceso completado!\n');