# Planificación de Trayectorias mediante Campos Potenciales Artificiales

**Asignatura:** Modelado y Control de Robots  
**Grado:** Informática Industrial y Robótica (UPV - Campus d'Alcoi)  
**Autora:** Alexa Milla Villavicencio  
**Curso:** 2026/2027  

## Descripción

Simulación en Unity de un planificador local de trayectorias basado en **Campos Potenciales Artificiales** para un robot móvil de tracción diferencial en un entorno hospitalario. El controlador combina:

* **Campo atractivo** con transición entre potencial cónico y parabólico.
* **Campo repulsivo** calculado mediante percepción local con un anillo de 24 sensores virtuales (`Physics.Raycast`).
* **Control cinemático no holonómico** que transforma la fuerza resultante en velocidades lineal y angular $(v, \omega)$ sobre el `Rigidbody`.
* **Detección y escape de mínimos locales** inspirado en el algoritmo *Randomized Potential Planner* (RPP) mediante caminatas aleatorias.

## Requisitos

* **Motor:** Unity 6.6 (`6000.6.8f1`).
* **Dependencias externas:** Ninguna. El proyecto utiliza únicamente componentes nativos de Unity y C#.

## Cómo ejecutar el proyecto

1. Clonar o descargar este repositorio.
2. Abrir la carpeta del proyecto desde **Unity Hub** con la versión de Unity indicada.
3. Cargar la escena principal en `Assets/Scenes/0.unity`.
4. Pulsar **Play** y visualizar la ejecución desde la pestaña **Scene** para observar en tiempo real los rayos de detección y los vectores de fuerza.

## Estructura principal

* **Script de control:** `Assets/PotentialFieldController.cs` (contiene el cálculo de fuerzas, el control no holonómico, los parámetros base y la lógica de escape RPP).
* **Objetos clave en la jerarquía (*Hierarchy*):**
  * `Robot`: plataforma móvil diferencial con `Rigidbody` y el script de control.
  * `Invisible_Cube_Target`: objeto que define la posición objetivo ($q_{\text{final}}$).
  * `MinimoLocalPared`: obstáculo frontal en el pasillo utilizado para forzar un mínimo local.

## Reproducción de las pruebas

Al abrir la escena, el proyecto viene configurado por defecto para ejecutar directamente la **Prueba 1** (`MinimoLocalPared` desactivado y el objetivo situado dentro de la habitación).

### Prueba 1: Navegación hacia una habitación y paso por puerta (configuración por defecto)

1. Verificar que `MinimoLocalPared` está **desactivado** en la jerarquía.
2. Verificar que `Invisible_Cube_Target` se encuentra en la posición: `(X = 45.87, Y = 0.05, Z = 23)`.
3. Pulsar **Play**. El robot bordeará la pared del pasillo, cruzará la puerta y se detendrá en el objetivo evitando la mesa.

### Prueba 2: Detección y escape de mínimo local (RPP)

1. **Activar** el objeto `MinimoLocalPared` en la jerarquía.
2. Cambiar únicamente la coordenada `X` de `Invisible_Cube_Target` a `25.3`, dejándolo en `(X = 25.3, Y = 0.05, Z = 23)` (detrás de la pared que genera el mínimo local).
3. Pulsar **Play**. Al quedar bloqueado frente a la pared, el controlador activará el modo de escape RPP (avisando por consola) y rodeará el obstáculo hasta alcanzar la meta.

## Leyenda de depuración visual (vista *Scene*)

* **Naranja:** Rayos `Physics.Raycast` que detectan obstáculos dentro de $\rho_0 = 2.5\text{ m}$.
* **Verde:** Fuerza atractiva ($F_{\text{att}}$).
* **Rojo:** Fuerza repulsiva ($F_{\text{rep}}$).
* **Azul:** Fuerza resultante ($F_{\text{total}}$).
* **Amarillo:** Dirección aleatoria durante el modo de escape RPP.

## Vídeos de demostración

* [Vídeo 1: Navegación hacia habitación y paso por puerta](https://youtu.be/watlJv_H-HE)
* [Vídeo 2: Escape de mínimo local mediante RPP](https://youtu.be/Awi4LjM7WzQ)