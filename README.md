# RUPU_TDMA
Implementacion de TDMA para la plataforma RUPU por Trabajo de Titulo

Notas de uso:

1. Al comenzar porner a los robots alineados con la linea blanca de de la pista para que se pueda determinar correctamente quien es el lider y quienes los seguidores.
2. El lider se asigna cuando su sensor mide una distancia mayor a 70cm. Esto se cambia en las lineas 248 y 249 del codigo principal. Si un robot no se reconoce como lider aunque no haya nada delante de el puede se debe poner un obstaculo a más de 70 cm pero a menos de 1.5 metros.
3. Para enviar datos a la ESP32 que actua como monitor, se debe cambiar la linea 128 para que tenga la MAC especifica del nodo monitor.
4. Para usar el codigo de python para guardar los datos, se debe poner el puerto serial al que esta conectada la ESP32 monitor y se debe tener cerrado el monitor serial en Arduino porque solo una aplicacion puede leer los datos a la vez.
5. Se noto que los robots reconocen mejor la linea cuando se encuentran en un cuarto con poca luz.
6. Hay ejemplos de funcionamiento en los siguientes videos: https://drive.google.com/file/d/1OYd6ECJRTCqVNiLFLKH5KJmmX3zUiDNY/view?usp=drive_link  y  https://drive.google.com/file/d/1ORjfzH6eYW2nWP5AKGz2DfB6xvhV5LJa/view?usp=drive_link
