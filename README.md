# Controlador SCARA desde cero (ESP32 + steppers)

Creado por **automaticartisan**, con la asistencia de **Claude Sonnet 5**
(Anthropic). Licenciado bajo la [Licencia MIT](./LICENSE).

## Cómo funciona

- **`Kinematics.h`** — cinemática directa e inversa de un brazo de 2 eslabones
  (SCARA clásico) usando ley de cosenos. `inverse()` te avisa si un punto está
  fuera del área de trabajo antes de intentar moverse ahí.
- **`TrapProfile.h`** — perfil de velocidad trapezoidal (rampa de aceleración,
  crucero a velocidad constante, rampa de desaceleración) por eje. Incluye
  `rescaleToTime()`, que estira el perfil en el tiempo sin cambiar la
  distancia recorrida: así se logra que varias juntas, cada una con su propio
  recorrido, empiecen y terminen exactamente al mismo tiempo.
- **`StepperAxis.h`** — genera los pulsos STEP/DIR mediante un acumulador de
  fase (oscilador controlado numéricamente) actualizado a frecuencia fija
  desde una interrupción de timer. También lleva la cuenta de la posición en
  pasos de cada eje.
- **`scara_controller.ino`** — configura el hardware, arranca el timer a
  20 kHz, calcula el tiempo maestro entre las tres juntas (J1, J2, Z) y
  expone un parser de comandos simple por Serial.

## Límites de software

Además de la limitación geométrica (que un punto sea alcanzable por la
cinemática del brazo), ahora cada solución de cinemática inversa se valida
contra un **rango seguro por eje**, calculado a partir de:

1. El **recorrido mecánico total** de ese eje (`J1_TRAVEL_RANGE_DEG`,
   `J2_TRAVEL_RANGE_DEG`, `Z_TRAVEL_RANGE_MM`) — todo lo que el eje puede
   moverse sin chocar nada.
2. Un **margen de seguridad respecto al switch de homing**
   (`J1_HOMING_MARGIN_DEG`, `J2_HOMING_MARGIN_DEG`, `Z_HOMING_MARGIN_MM`) —
   para que el robot no ande rozando/disparando el switch en cada
   movimiento cerca de ese extremo durante el uso normal.

El switch de cada eje queda en el extremo hacia el que apunta su
`HOMING_DIR`. A partir de la posición conocida de homing (`*_HOMING_ANGLE_DEG`
/ `Z_HOMING_MM`), el límite del lado del switch se calcula retrocediendo el
margen, y el límite del lado lejano recorriendo el rango completo hacia el
otro extremo — el código resuelve solo cuál de los dos es el mínimo y cuál
el máximo según el signo de `HOMING_DIR`, así que no hay que pensarlo a mano.

Este chequeo se aplica en **los mismos puntos** donde antes solo se miraba
si un punto era cinemáticamente alcanzable: el destino de un `G0`, el
destino de un `G1`, los ~12 puntos muestreados de un `G2`/`G3` antes de
arrancarlo, y como red de seguridad en cada tick de interpolación mientras
un movimiento está en marcha. Si algo lo viola, se imprime `ERROR: punto
fuera de los límites de software` y el comando se rechaza (o el movimiento
en curso se detiene, si el chequeo falla ya en marcha — no debería pasar si
la validación previa funcionó, pero queda como respaldo).

**`CALIBRATE` es la única rutina que se salta este chequeo a propósito** —
necesita poder llegar hasta el switch para homear, que por definición está
más allá del margen de seguridad.

Antes de usarlo, ajustá en el .ino:

| Parámetro | Qué es |
|---|---|
| `J1_TRAVEL_RANGE_DEG`, `J2_TRAVEL_RANGE_DEG`, `Z_TRAVEL_RANGE_MM` | Recorrido mecánico seguro total de cada eje |
| `J1_HOMING_MARGIN_DEG`, `J2_HOMING_MARGIN_DEG`, `Z_HOMING_MARGIN_MM` | Cuánto quedarse lejos del switch durante la operación normal |

Sugerencia: si `HOMING_CLEARANCE_STEPS` (el retiro final que hace
`CALIBRATE` tras homear) deja al eje más cerca del switch que el margen de
software correspondiente, el robot arrancará ya "pegado" al límite. Conviene
que el margen de software sea un poco mayor que ese retiro.

## Calibración (homing con switches)

`CALIBRATE` corre un homing de dos etapas en J1, J2 y Z, en ese orden:

1. **Acercamiento rápido** hasta que el switch de ese eje se dispara.
2. **Retroceso** hasta soltarlo, más un poco de carrera extra.
3. **Reacercamiento lento** — el segundo toque, mucho más lento, es el que
   realmente se usa para calibrar (así el punto de disparo es repetible;
   a alta velocidad la inercia y el tiempo de respuesta del switch hacen que
   varíe más de lo que conviene).
4. Se fija la posición conocida (`J1_HOMING_ANGLE_DEG`, `J2_HOMING_ANGLE_DEG`,
   `Z_HOMING_MM` — **no tienen por qué ser 0**, poné el valor real de tu
   montaje) exactamente en ese punto, y el eje se aleja un poco del switch
   para no dejarlo presionado.

Es **bloqueante**: mientras corre, no se procesan otros comandos ni la cola.
Esto es intencional (no tiene sentido aceptar movimientos mientras no se
sabe dónde está el brazo), y funciona porque el generado de pulsos vive en
la interrupción de hardware del timer — sigue corriendo aunque `loop()` esté
esperando en un `while`.

**Antes de usarlo, ajustá en el .ino:**

| Parámetro | Qué es |
|---|---|
| `J1_LIMIT_PIN`, `J2_LIMIT_PIN`, `Z_LIMIT_PIN` | Pines de los switches. Usan `INPUT_PULLUP` (switch a GND = activado); evitá GPIO34-39 en ESP32, no tienen pull-up interno |
| `J1_HOMING_ANGLE_DEG`, `J2_HOMING_ANGLE_DEG`, `Z_HOMING_MM` | La posición real y conocida de tu robot en el punto donde cada switch se dispara |
| `HOMING_DIR_J1/J2/Z` | +1 o -1: hacia qué lado hay que moverse para acercarse al switch de cada eje (depende de dónde lo montaste) |
| `HOMING_FAST_FRACTION` / `HOMING_SLOW_FRACTION` / `HOMING_BACKOFF_FRACTION` | Velocidades de homing, como fracción de la VMAX normal de cada eje |

Si al correr `CALIBRATE` un eje se aleja del switch en vez de acercarse,
invertí su `HOMING_DIR_*`. Si tarda 15 segundos y aborta con "timeout",
revisá cableado, o que el switch efectivamente conecte a GND al activarse.

Nota: `HOME` (redefinir el origen como la posición actual, sin switches) y
`CALIBRATE` (homing real con switches, posición absoluta conocida) son cosas
distintas — `HOME` sigue sirviendo para pruebas rápidas sin hardware de
homing instalado.

## Leer y fijar la posición cartesiana desde código

Dos funciones reutilizables, pensadas para llamarse desde cualquier parte
del `.ino` (no solo desde el parser de comandos):

- **`Pose getCurrentPose()`** — cinemática directa a partir de la posición
  REAL de los ejes (cuenta de pasos), no de `currentTargetPose` (que es el
  último destino *comandado*, y puede no coincidir mientras un movimiento
  está en curso). `reportPosition()` y `runCalibration()` ya la usan
  internamente.
- **`bool setCurrentPose(const Pose &p, ElbowConfig elbow = ELBOW_UP)`** —
  la versión cartesiana de `HOME`: fija los pasos de los tres ejes para que
  correspondan a `p`, sin mover ningún motor. Devuelve `false` (con mensaje
  de error) si `p` no es alcanzable, o si hay un movimiento en marcha o en
  cola (para evitar redefinir la posición mientras el controlador está a
  mitad de un tramo). A propósito **no** valida contra los límites de
  software, igual que `HOME`/`CALIBRATE` — describe dónde está el brazo de
  verdad, que podría estar más allá del rango operativo normal.

Expuesta por Serial como `SETPOS X<mm> Y<mm> Z<mm>` (ejes no especificados
mantienen su valor actual). Útil, por ejemplo, para decirle al controlador
dónde está el brazo después de reposicionarlo a mano, o para aplicar un
offset de calibración fino sin tener que re-correr todo `CALIBRATE`.

## Encolado de movimientos: no frenar del todo entre tramos

Antes, cada G0/G1/G2/G3 se ejecutaba al instante y el siguiente comando
esperaba a que terminara del todo (parado en cero). Ahora los comandos se
**encolan** (buffer de 8) y se van despachando en orden apenas el tramo
anterior termina — y, para G1/G2/G3 consecutivos, sin frenar a cero en el
medio si la geometría lo permite.

Cómo funciona:

1. Cada `TrapProfile` ahora acepta velocidad de **entrada y salida**
   distintas de cero (antes siempre arrancaba y terminaba en reposo). Si la
   distancia del tramo no alcanza para lograr la velocidad de salida pedida,
   el perfil calcula la mejor alcanzable (`getEndVelocity()`) en vez de
   fallar.
2. Al despachar un tramo G1/G2/G3, su velocidad de **entrada** (`v0`) es la
   velocidad real con la que terminó el tramo anterior (`carryVelocity`).
3. Su velocidad de **salida** objetivo (`v1`) se calcula mirando el
   siguiente comando en la cola (sin ejecutarlo todavía): se compara la
   dirección de avance al final de este tramo con la dirección de avance al
   principio del siguiente, y de ahí sale una "velocidad de esquina" segura
   — si el ángulo es suave (casi seguir derecho), pasa a full velocidad; si
   es un giro cerrado, frena bastante o del todo. Es la misma idea de
   "junction deviation" que usan GRBL/Marlin (parámetro `JUNCTION_DEVIATION_MM`,
   más alto = curvas más rápidas pero con más desviación real del vértice).
4. `G0` siempre rompe la cadena (arranca y termina en reposo), igual que un
   comando fallido (destino inalcanzable) — en ese caso se avisa por Serial
   y se sigue con el siguiente de la cola en vez de trabarse.

**Limitaciones de esta implementación** (a diferencia de un planificador
completo tipo GRBL, que mira TODA la cola hacia adelante y hacia atrás):

- Solo se mira **un** movimiento hacia adelante (look-ahead de 1), no toda
  la cola. Para una secuencia larga de tramos cortos en la misma dirección,
  un planificador completo podría acelerar de forma más óptima a lo largo
  de varios tramos; acá cada par se resuelve de forma local.
- Al arrancar cada tramo hay un posible "bache" de velocidad de hasta un
  tick de interpolación (~2.5 ms a 400 Hz) mientras el primer cálculo de
  velocidad se pone al día — imperceptible en la práctica, pero vale
  mencionarlo.
- Si un comando falla al despacharse, el punto de referencia (`currentTargetPose`)
  ya había asumido ese destino al encolarlo, así que el próximo comando que
  no especifique todos los ejes (X/Y/Z) podría partir de un punto que el
  brazo nunca alcanzó realmente. Evita encadenar comandos con ejes
  implícitos justo después de uno que sepas que puede fallar.

Comando nuevo: `CLEAR` vacía la cola (no aborta el tramo que ya esté en
marcha). `?` ahora también reporta si el controlador está ocupado y cuántos
comandos quedan en la cola.

## Tres tipos de movimiento: G0 (juntas), G1 (línea recta) y G2/G3 (arco)

**G0 — sincronizado en espacio de juntas.** Cada eje calcula cuánto tardaría
en llegar a su destino usando sus propios límites de velocidad/aceleración.
El eje más lento fija el "tiempo maestro"; los demás reescalan su perfil
(misma distancia, más tiempo) y arrancan todos juntos, así llegan
simultáneamente. Es más simple y barato de calcular, pero la punta del
brazo describe una **curva**, no una recta, entre origen y destino.

**G1 — interpolación lineal cartesiana real.** Aquí sí la punta del brazo
sigue una línea recta. Cómo funciona:

1. Se calcula un único perfil trapezoidal sobre la **longitud de la línea**
   (en mm), con los límites `LINEAR_VMAX_MMPS`/`LINEAR_AMAX_MMPS2`.
2. A `INTERP_FREQ_HZ` (400 Hz por defecto) — mucho más lento que la ISR de
   pasos, porque aquí sí se ejecuta trigonometría (`acos`, `atan2`, etc.) —
   se evalúa en qué punto de la recta debería estar el brazo en ese instante
   (`positionAt()` del perfil), y se resuelve la cinemática inversa para ese
   punto.
3. Se compara la posición articular objetivo (en pasos) con la del tick
   anterior, y esa diferencia dividida por el intervalo da la velocidad que
   debe llevar cada eje **hasta el próximo tick**.
4. Esa velocidad se le pasa a cada `StepperAxis` (modo `VELOCITY_TRACK`), y
   es la ISR de 20 kHz la que efectivamente reparte esos pasos de forma
   suave dentro del intervalo — el tick de interpolación solo actualiza
   "a qué velocidad" cada pocos milisegundos.

**G2 (horario) / G3 (antihorario) — arcos reales, estilo G-code.** Reutiliza
exactamente el mismo mecanismo que G1: lo único que cambia es `poseAtFraction()`,
que en vez de interpolar linealmente entre dos puntos, calcula
`centro + radio·(cos,sin)` del ángulo correspondiente a la fracción de
recorrido. `I`/`J` dan la posición del **centro del arco relativa al punto de
inicio** (igual que en G-code estándar); el radio se calcula solo, y se
valida que el punto final quede a la misma distancia del centro que el
inicial (si no, es un error de I/J). Si `Z` cambia entre inicio y fin, el
resultado es una hélice (Z avanza de forma lineal junto con el ángulo).

Antes de arrancar un arco se muestrean ~12 puntos a lo largo de todo el
recorrido (no solo los extremos) para comprobar que son alcanzables — a
diferencia de una línea, la distancia del arco al origen del brazo puede
variar de forma no monótona, así que un arco con extremos válidos podría
igual cruzar una zona inalcanzable en el medio.

**Cuidado con las singularidades (aplica a G1 y G2/G3):** cerca de la
extensión máxima del brazo o muy plegado, una junta puede necesitar girar
muy rápido para mover la punta apenas un poco (es geometría, no un error del
código). El controlador recorta la velocidad de cada eje a su `VMAX`
configurado y avisa una vez por Serial (`AVISO: un eje llegó a su velocidad
máxima...`) si eso ocurre; en ese caso el camino dejará de seguirse con
precisión en ese tramo. Si te pasa seguido, baja
`LINEAR_VMAX_MMPS`/`LINEAR_AMAX_MMPS2` o evita pasar cerca del centro/límite
del área de trabajo.

## Antes de usarlo — calibra estos valores en el .ino

| Parámetro | Qué es |
|---|---|
| `L1_MM`, `L2_MM` | Longitud real de tus dos eslabones |
| `STEPS_PER_REV_J1/J2` | Pasos por vuelta del motor × microstepping ÷ reducción mecánica de cada junta |
| `STEPS_PER_MM_Z` | Pasos por milímetro del eje Z (según husillo/correa/polea) |
| Pines `*_STEP_PIN`, `*_DIR_PIN` | Según tu cableado a los drivers |
| `*_VMAX_SPS`, `*_AMAX_SPS2` | Velocidad/aceleración máxima segura de cada motor, en pasos/s y pasos/s² |
| `*_LIMIT_PIN`, `*_HOMING_ANGLE_DEG`/`Z_HOMING_MM`, `HOMING_DIR_*` | Ver sección "Calibración" más abajo |

Si usas un TB6560 como driver (como en tu otro proyecto), recuerda que sus
dip switches definen el microstepping, lo que cambia `STEPS_PER_REV`.

## Comandos por Serial (115200 baudios)

```
G1 X0 Y100 F40      -> a la cola
G1 X100 Y100 F40    -> a la cola; si el giro con el tramo anterior es suave,
                        no frena del todo antes de tomarlo
G1 X100 Y0 F40       -> a la cola
```

Los tres se ejecutan uno tras otro sin esperar confirmación por Serial entre
medio (podés mandarlos seguidos). Ejemplos de comandos individuales:

```
CALIBRATE                          -> homing con switches en J1, J2 y Z
SETPOS X150 Y0 Z20                 -> redefine la posición actual SIN mover motores
G0 X100 Y50 Z10                  -> movimiento rápido, sincronizado en juntas (camino curvo)
G1 X100 Y50 Z10 F40              -> movimiento LINEAL a 40 mm/s (línea recta real; F opcional)
G2 X100 Y0 I0 J-50 F30           -> arco HORARIO, centro 50mm por debajo del punto de inicio
G3 X100 Y0 I0 J-50 F30           -> mismo arco pero ANTIHORARIO
CLEAR                             -> vacía la cola de comandos pendientes
HOME                              -> define la posición actual como origen (0,0,0 en pasos)
?                                  -> reporta posición actual, si está ocupado, y cuántos hay en cola
```

## Compatibilidad del core ESP32

La API de timers de hardware cambió entre versiones del paquete ESP32 para
Arduino: el código trae la versión "clásica" (core < 3.0) activa y la
versión nueva (core 3.x, basada en esp-idf 5.x) comentada justo debajo, en
`setup()`. Revisa qué versión tienes instalada en el Boards Manager del IDE
y usa el bloque correspondiente.

## Por qué el timer va a 20 kHz

El acumulador de fase suma `velocidad_pasos_por_seg * dt` en cada tick, y
emite un pulso STEP cuando cruza 1.0. Con `dt = 50 µs`, mientras la velocidad
máxima de cada eje no supere unos ~10000 pasos/s el acumulador nunca avanza
más de un paso por tick, así que no se pierden pasos por redondeo. Si tus
motores necesitan velocidades más altas, sube `ISR_FREQ_HZ`.

## Próximos pasos posibles

- Homing simultáneo (los tres ejes a la vez en vez de uno por uno) para
  calibrar más rápido
- Planificador de look-ahead completo (mirar toda la cola, no solo el
  siguiente tramo) para un encolado más óptimo en secuencias largas
- Comando de parada de emergencia que aborte el tramo en marcha, no solo
  vacíe la cola pendiente (`CLEAR` actual)
- Límite de software para la posición cartesiana (X/Y/Z) directamente, además
  del actual por ángulo/altura de cada eje — útil si querés, por ejemplo,
  evitar una zona rectangular de la mesa de trabajo
- Añadir una 4ª junta (rotación de muñeca) — el patrón ya generaliza,
  solo hace falta otro `StepperAxis` + un ángulo más en `JointAngles`
