%--------------------------------------------------------------------------
% SCRIPT PARA LA CARACTERIZACIÓN DE UN SERVOMOTOR DYNAMIXEL (v2)
% AJUSTADO PARA PUERTO COM5 Y VALIDADO CON FUNCIONES SDK PROPORCIONADAS
%--------------------------------------------------------------------------

% Cargar la librería del SDK de Dynamixel. Esta sección es clave y asegura
% que MATLAB pueda encontrar las funciones como 'write4ByteTxRx'.
lib_name = '';
if strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c'; % Para Windows de 64 bits
elseif strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c'; % Para Windows de 32 bits
else
    error('Sistema operativo no compatible con este script. Adapte el nombre de la librería.');
end

% Solo carga la librería si no ha sido cargada antes
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%==========================================================================
% 1. CONFIGURACIÓN DE LA COMUNICACIÓN (Valores Principales)
%==========================================================================
PROTOCOL_VERSION    = 2.0;
DXL_ID              = 1;            % <-- AJUSTAR: ID de tu servomotor
BAUDRATE            = 57600;        % <-- AJUSTAR: Baudrate de tu servomotor
DEVICENAME          = 'COM5';       % <-- CONFIRMADO: Puerto COM especificado

% Direcciones del "Control Table" para un Dynamixel XM430.
% Es crucial verificar que estas direcciones sean correctas para tu modelo.
ADDR_TORQUE_ENABLE  = 64;   % Habilita o deshabilita el torque
ADDR_GOAL_POSITION  = 116;  % Posición a la que se desea mover el motor
ADDR_PRESENT_POSITION = 132;% Posición actual del motor
ADDR_OPERATING_MODE = 11;   % Para cambiar entre modo posición, velocidad, etc.

% Constantes para una mejor legibilidad del código
TORQUE_ENABLE       = 1;
TORQUE_DISABLE      = 0;
POSITION_MODE       = 3;    % Modo de control por posición

% Inicializar manejadores de puerto y paquete. Son como los "drivers".
port_num = portHandler(DEVICENAME);
packetHandler();

% Abrir puerto y establecer baudrate
if (openPort(port_num))
    fprintf('Puerto COM5 abierto exitosamente.\n');
else
    unloadlibrary(lib_name);
    error('ERROR: No se pudo abrir el puerto COM5. ¿Está conectado o en uso por otro programa?');
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
% Se establece el modo de operación a "Position Control"
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, POSITION_MODE);
fprintf('Modo de operación configurado en "Control de Posición".\n');

% Habilitar el Torque del servomotor para que mantenga la posición
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
fprintf('Torque habilitado. ¡El motor ahora opondrá resistencia al movimiento manual!\n');

%==========================================================================
% 3. EJECUCIÓN DEL EXPERIMENTO (Respuesta al Escalón)
%==========================================================================
% Definimos la entrada escalón de posición.
% El motor se moverá de pos_inicial a pos_final.
% Los valores están en "ticks" del encoder (0 a 4095 para 360 grados).
pos_inicial = 1024; % Aproximadamente 90 grados
pos_final   = 2048; % Aproximadamente 180 grados

% Mover a la posición inicial y esperar a que se estabilice
fprintf('Moviendo a la posición inicial (%d ticks)...\n', pos_inicial);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, pos_inicial);
pause(2); % Espera de 2 segundos para asegurar que el motor llegue y se asiente.

% Preparación para la captura de datos
fprintf('Iniciando captura de datos para la respuesta al escalón...\n');
duracion_experimento = 2; % [s] Duración total de la captura
Ts_deseado = 0.01;        % [s] Tiempo de muestreo (10 ms -> 100 Hz)
num_muestras = floor(duracion_experimento / Ts_deseado);

tiempo = zeros(num_muestras, 1);
posicion_real = zeros(num_muestras, 1);
% La señal de entrada es un escalón perfecto que va de 0 al valor final
entrada_escalon = ones(num_muestras, 1) * (pos_final - pos_inicial);

% --- ¡APLICACIÓN DEL ESCALÓN Y CAPTURA DE DATOS! ---
tic; % Iniciar cronómetro

% Se envía el comando de la nueva posición (el escalón)
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, pos_final);

% Bucle para leer la posición del motor a intervalos regulares
for i = 1:num_muestras
    % Leer la posición actual del motor. Esta es la función clave de lectura.
    posicion_real(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    tiempo(i) = toc; % Guardar el instante de tiempo exacto
    
    % Pausa activa para mantener el tiempo de muestreo lo más constante posible
    while(toc < i * Ts_deseado)
        % No hacer nada, solo esperar
    end
end

fprintf('Captura de datos finalizada. Se tomaron %d muestras.\n', num_muestras);

%==========================================================================
% 4. ANÁLISIS DE DATOS CON SYSTEM IDENTIFICATION TOOLBOX
%==========================================================================
% La respuesta del sistema se mide como el cambio desde la posición inicial.
respuesta_sistema = posicion_real - pos_inicial;
entrada_escalon = ones(num_muestras, 1) * (pos_final - pos_inicial);

% Crear un objeto 'iddata', que es el formato estándar del toolbox.
datos_exp = iddata(respuesta_sistema, entrada_escalon, Ts_deseado);
datos_exp.InputName = 'Escalón de Posición (ticks)';
datos_exp.OutputName = 'Respuesta del Motor (ticks)';

% Estimar la función de transferencia de primer orden ('np1' = 1 polo, 0 ceros)
fprintf('Estimando la función de transferencia con "tfest"...\n');
sys_estimado = tfest(datos_exp, 1, 0);

disp('----------------------------------------------------');
disp('Función de Transferencia Estimada G(s):');
disp(sys_estimado);

% --- CORRECCIÓN DEL ERROR ---
% Extraemos los coeficientes del numerador y denominador directamente.
% En lugar de usar .Kp, calculamos los parámetros del modelo G(s) = A / (s + B)
num_coeffs = sys_estimado.Numerator;      % Coeficiente A
den_coeffs = sys_estimado.Denominator;    % Coeficientes [1, B]

A = num_coeffs;
B = den_coeffs(2);

% Convertimos a la forma estándar G(s) = K / (T*s + 1)
K = A / B;  % La ganancia estática
T = 1 / B;  % La constante de tiempo

fprintf('\nParámetros del Modelo de Primer Orden G(s) = K / (T*s + 1):\n');
fprintf('  -> Ganancia Estática (K): %.4f\n', K);
fprintf('  -> Constante de Tiempo (T): %.4f segundos\n', T);
disp('----------------------------------------------------');
%==========================================================================
% 5. VISUALIZACIÓN DE RESULTADOS
%==========================================================================
figure;
compare(datos_exp, sys_estimado); % Comando que grafica la curva real vs. la del modelo
title('Comparación: Respuesta Experimental vs. Modelo de 1er Orden Estimado');
legend('Respuesta Experimental Real', 'Respuesta del Modelo Matemático');
xlabel('Tiempo (s)');
ylabel('Posición (ticks desde el inicio)');
grid on;
set(gcf, 'color', 'w'); % Fondo blanco para la figura

%==========================================================================
% 6. FINALIZACIÓN SEGURA
%==========================================================================
% Es una buena práctica deshabilitar el torque y siempre cerrar el puerto.
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
closePort(port_num);
unloadlibrary(lib_name);
fprintf('\nComunicación finalizada y torque deshabilitado. ¡Proceso completado!\n');