# Crane x7 Robot
Trabajo final de Cinemática y Dinámica, diplomado en Robótica, UCB

Desarrollado con ROS Noetic y Ubuntu 20.04


## 1. Creación de "workspace"
Luego de installar ROS. Siga los siguientes pasos:
1. Cree una carpeta vacia para su workspace
2. Dentro cree otra carpeta llamada "src"
3. Dentro de la carpeta de su workspace

> worspace_file/

ejecute el siguiente comando en la terminal:
```
catkin_make
```
El comando lo configura la carpeta como workspace. 

Las carpetas tipo package van dentro de la carpeta src. Es decir:

> worspace_file/src/package_file

## 2. Clonar Github

Este github es un package de ROS, para clonarlo sin afectar otros packages cree una carpeta llamada "trabajo_final", es decir:

> worspace_file/src/trabajo_final

Clone el repositorio dentro de la carpeta "trabajo_final".

En una terminal vaya a:
> worspace_file/

Y ejecute los siguientes comandos:
```
source devel/setup.bash
catkin_make
```

## 3. Cinemática directa
Inicie roscore, en una terminal, sin importar la direccion corra:

```
roscore
```
Para probar la cinematica directa vaya a:
> worspace_file/

Y en una terminal corra el siguiente comando:
```
roslaunch trabajo_final trabajo_final.launch 
```
Se abrira la interfaz de RVIZ con una ventana GUI para mover el robot.

Abra la posición del efector final:

![alt text](https://github.com/WilberRojas/ROS-Crane-x7-Robot/blob/main/images/RVIZ_endeffector.png)

Verifique que los datos son los mismos que los calculados, para ello en otra terminal corra:

```
rostopic echo /position
```
La posicion deberia aproximarse por varios decimales.

![alt text](https://github.com/WilberRojas/ROS-Crane-x7-Robot/blob/main/images/RVIZ_vs_terminal.png)

## 3.1. Demostración

https://user-images.githubusercontent.com/74274632/170376920-0d4ac1fb-b261-40ed-9a0d-32aae3eff3b8.mp4

## 4. Cinemática Inversa

Para la cinemática inversa, tambien debe correr roscore en una terminal

```
roscore
```

Igual al anterior, en una terminal vaya a:
> worspace_file/

Y corra el siguiente comando:

```
roslaunch trabajo_final inverse_cranex7.launch
```

Se abrira la interfaz de RVIZ, esta vez sin la ventana GUI. 

Al ser cinemática inversa debemos mandarle al robot la posición a la que se debe mover. 

Entonces en una terminal escribimos lo siguiente:

```
rostopic pub /position geometry_msgs/Point "x: 0.55
y: 0.0
z: 0.0" 
```

x,y,z es pa posición espacial a la que el efector final debe moverse.

Nota: Si se ingresa un valor inalcanzable el robot nunca se estabilizara.

El robot tiene un radio alcanzable de aproximadamente 0.6 unidades. 

## 4.1. Demostración


https://user-images.githubusercontent.com/74274632/170377007-1a2af094-314d-4137-8a2f-151aeedf40ae.mp4
