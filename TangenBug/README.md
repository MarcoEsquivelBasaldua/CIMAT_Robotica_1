# Tangent Bug
El algoritmo de planeaci'on de movimientos *TangentBug* en rob'otica m'ovil, determina el camino mas corto a la meta para un robot puntual usando un sensor, con rango de alcance, de orientaci'on de 360 grados de resolucion infinita, idealmente.<br />
En esta implementaci'on se hace uso del simulador *webots* en el que se simula el robot *e-puck*, que es un robot tipo disco, adaptando el agoritmo con 8 rayos de sensado radiales. Entre las modificaciones agregadas, se tienen funciones que hacen rotar al robot en su sitio para alinearse ya sea a la meta o a una ruta tangente al obst'aculo que est'e siguiendo. 
La ejecuci'on de la simulaci'on se observa en el video adjunto en el que la meta es marcada por un punto rojo.
