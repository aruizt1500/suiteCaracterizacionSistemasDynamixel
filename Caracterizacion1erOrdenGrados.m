% 

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

% Inicializar manejadores de puerto y paquete. 
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
% 3. EJECUCIÓN DEL EXPERIMENTO (GRADOS)
%==========================================================================
% Factor de conversión de ticks a grados
ticks_a_grados = 360.0 / 4096.0;

% Definimos la entrada escalón en GRADOS.
pos_inicial_grados = 90.0;
pos_final_grados = 180.0;

% Convertimos a ticks para enviar los comandos al motor
pos_inicial_ticks = round(pos_inicial_grados / ticks_a_grados);
pos_final_ticks = round(pos_final_grados / ticks_a_grados);

% (El resto del código de movimiento y captura de datos permanece igual,
% usando las variables _ticks para los comandos write4ByteTxRx)
% ...
% Mover a la posición inicial...
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, pos_inicial_ticks);
% ...
% Aplicar el escalón...
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, pos_final_ticks);
% ...
% (El bucle de captura sigue leyendo los ticks de posicion_real)
% ...

%==========================================================================
% 4. ANÁLISIS DE DATOS (AHORA EN GRADOS)
%==========================================================================
% --- ¡AQUÍ ESTÁ LA MAGIA! ---
% Convertimos los datos capturados (en ticks) a grados ANTES del análisis.
posicion_real_grados = posicion_real * ticks_a_grados;

% La respuesta del sistema ahora se calcula en grados.
respuesta_sistema_grados = posicion_real_grados - pos_inicial_grados;
entrada_escalon_grados = ones(num_muestras, 1) * (pos_final_grados - pos_inicial_grados);

% Crear el objeto 'iddata' con los datos en grados
datos_exp = iddata(respuesta_sistema_grados, entrada_escalon_grados, Ts_deseado);
datos_exp.InputName = 'Escalón de Posición (grados)';
datos_exp.OutputName = 'Respuesta del Motor (grados)';

% (El resto del código de estimación y cálculo de K y T es exactamente el mismo)
% ...
sys_estimado = tfest(datos_exp, 1, 0);
% ...

%==========================================================================
% 5. VISUALIZACIÓN DE RESULTADOS (LOS EJES ESTARÁN EN GRADOS)
%==========================================================================
figure;
compare(datos_exp, sys_estimado);
title('Comparación: Respuesta Experimental vs. Modelo de 1er Orden');
legend('Respuesta Experimental Real', 'Respuesta del Modelo Matemático');
xlabel('Tiempo (s)');
ylabel('Posición (grados desde el inicio)'); % <-- Eje Y ahora en grados
grid on;