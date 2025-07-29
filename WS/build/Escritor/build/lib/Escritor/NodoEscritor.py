import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen
import time

velocidad_lineal = 2.0  # Velocidad lineal de la tortuga (2.0 pixel/s)
tiempo_movimiento = 0.5  # Tiempo de movimiento en segundos
velocidad_angular = 3.1416  # Velocidad angular para que pase de horizontal a vertical en 0.5 segundos (rad/s, pi es 90º porque el tiempo es 1/2 s)


class NodoEscritor(Node):
	def __init__(self):
		super().__init__('nodo_escritor')
		
		self.posicion_inicial = 0.5
		self.posicion_actual = self.posicion_inicial

		self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
		time.sleep(1.0)  # Esperar a que turtlesim esté listo
		
		while True:
			word = input("Introduce la palabra a dibujar (máx 10 letras): ").upper()
			if len(word) <= 10:
				break
			self.get_logger().info("La palabra debe tener un máximo de 10 letras. Inténtalo de nuevo.")

		self.get_logger().info(f"Palabra a dibujar: {word}")

		self.teleport(self.posicion_inicial, 5.5, 0.0) # Rango de la tortuga (0.0,11.0) No usar los valores límites

		self.write_word(word)

		self.stop()

	
	def write_word(self, word):
		for letra in word:
			self.get_logger().info(f"Dibujando letra: {letra}")

			if letra == 'B':
				self.draw_B()
			elif letra == 'C':
				self.draw_C()
			elif letra == 'E':
				self.draw_E()
			elif letra == 'O':
				self.draw_O()
			elif letra == 'R':
				self.draw_R()
			elif letra == 'S':
				self.draw_S()
			elif letra == 'U':
				self.draw_U()
			else:
				self.get_logger().info(f'Letra {letra} no implementada')


	def move(self, linear=0.0, angular=0.0, duration=1.0):
		twist = Twist()
		twist.linear.x = linear
		twist.angular.z = angular
		end_time = time.time() + duration
		while time.time() < end_time:
			self.publisher.publish(twist)
			time.sleep(0.1)

	def teleport(self, x, y, theta=0.0):
		client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
		while not client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Esperando al servicio de teletransporte...')
		
		self.set_pen(0, 0, 0, 1, 1) # Levanta el lápiz
		
		req = TeleportAbsolute.Request()
		req.x = x
		req.y = y
		req.theta = theta

		future = client.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		self.set_pen(0, 0, 0, 3, 0) # Baja el lápiz
		time.sleep(0.05)

	def set_pen(self, r=0, g=0, b=0, width=3, off=1):
		client = self.create_client(SetPen, '/turtle1/set_pen')
		while not client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Esperando al servicio set_pen...')

		req = SetPen.Request()
		req.r = r
		req.g = g
		req.b = b
		req.width = width
		req.off = off  # 1 = pen up (no dibuja), 0 = pen down

		future = client.call_async(req)
		rclpy.spin_until_future_complete(self, future)

	def tp(self):

		self.stop()

		self.posicion_actual = self.posicion_actual + 1.025
		if self.posicion_actual < 11.0:
			self.teleport(self.posicion_actual, 5.5, 0.0)

	def stop(self):
		self.publisher.publish(Twist())  # Para la tortuga
		time.sleep(0.5)

	def draw_ref(self):
		# Parte vertical
		self.set_pen(0, 0, 0, 3, 0) # Baja el lápiz
		self.move(velocidad_lineal, 0.0, tiempo_movimiento)

		self.set_pen(255, 0, 0, 3, 0) # Baja el lápiz
		self.move(velocidad_lineal, 0.0, tiempo_movimiento)

		self.set_pen(0, 0, 0, 3, 0) # Baja el lápiz
		self.move(velocidad_lineal, 0.0, tiempo_movimiento)
		
		self.set_pen(255, 0, 0, 3, 0) # Baja el lápiz
		self.move(velocidad_lineal, 0.0, tiempo_movimiento)
	
		self.set_pen(0, 0, 0, 3, 0) # Baja el lápiz
		self.move(velocidad_lineal, 0.0, tiempo_movimiento)
		
		self.set_pen(255, 0, 0, 3, 0) # Baja el lápiz
		self.move(velocidad_lineal, 0.0, tiempo_movimiento)
	
		self.set_pen(0, 0, 0, 3, 0) # Baja el lápiz
		self.move(velocidad_lineal, 0.0, tiempo_movimiento)
		
		self.set_pen(255, 0, 0, 3, 0) # Baja el lápiz
		self.move(velocidad_lineal, 0.0, tiempo_movimiento)
	
		self.set_pen(0, 0, 0, 3, 0) # Baja el lápiz
		self.move(velocidad_lineal, 0.0, tiempo_movimiento)
	
		self.set_pen(255, 0, 0, 3, 0) # Baja el lápiz
		self.move(velocidad_lineal, 0.0, tiempo_movimiento)

		self.teleport(self.posicion_inicial, 5.5, 0.0) # Rango de la tortuga (0.0,11.0) No usar los valores límites


	def draw_R(self):
		# Vertical
		self.move(0.0, velocidad_angular, tiempo_movimiento)	
		self.move(velocidad_lineal, 0.0, 1.05*tiempo_movimiento)
		
		# Mini horizontal
		self.move(0.0, -velocidad_angular, tiempo_movimiento)	
		self.move(velocidad_lineal/4, 0.0, tiempo_movimiento)	

		# Curva
		self.move(velocidad_lineal/2, -velocidad_angular, 2*tiempo_movimiento)
		
		# Mini horizontal
		self.move(velocidad_lineal/4, 0.0, tiempo_movimiento)

		# Diagonal
		self.move(0.0, 3*velocidad_angular/2, tiempo_movimiento)	
		self.move(velocidad_lineal/2, 0.0, 1.05*tiempo_movimiento)

		self.tp()

	def draw_O(self):

		self.teleport(self.posicion_actual + 0.3, 5.5, 0.0)

		# Curva
		self.move(velocidad_lineal*1.05, 2*velocidad_angular, 2*tiempo_movimiento)
		

		self.tp()

	def draw_B(self):
		# Vertical
		self.move(0.0, velocidad_angular, tiempo_movimiento)	
		self.move(velocidad_lineal, 0.0, 1.05*tiempo_movimiento)
		
		# Mini horizontal
		self.move(0.0, -velocidad_angular, tiempo_movimiento)	
		self.move(velocidad_lineal/4, 0.0, tiempo_movimiento)	

		# Curva
		self.move(velocidad_lineal/2, -velocidad_angular, 2*tiempo_movimiento)
		self.move(0.0, 2*velocidad_angular, tiempo_movimiento)
		self.move(velocidad_lineal/2, -velocidad_angular, 2*tiempo_movimiento)

		# Mini horizontal
		self.move(velocidad_lineal/4, 0.0, tiempo_movimiento)
		self.tp()

	
	def draw_E(self):
		# Vertical
		self.move(0.0, velocidad_angular, tiempo_movimiento)	
		self.move(velocidad_lineal, 0.0, 1.05*tiempo_movimiento)
		
		# Horizontal
		self.move(0.0, -velocidad_angular, tiempo_movimiento)	
		self.move(velocidad_lineal/2, 0.0, tiempo_movimiento)	
		
		self.teleport(self.posicion_actual, 6.1, 0.0)
		
		# Mini horizontal
		self.move(velocidad_lineal/4, 0.0, tiempo_movimiento)
		self.teleport(self.posicion_actual, 5.5, 0.0)

		# Horizontal
		self.move(velocidad_lineal/2, 0.0, tiempo_movimiento)	

		self.tp()

	def draw_S(self):
		self.teleport(self.posicion_actual + 0.1, 5.5, 0.0)

		# Curva
		self.move(velocidad_lineal, 1.8*velocidad_angular, tiempo_movimiento)
		self.move(velocidad_lineal, -2*velocidad_angular, tiempo_movimiento)

		self.tp()
		
	def draw_C(self):
		self.teleport(self.posicion_actual + 0.7, 6.7, 3.1416)

		# Curva
		self.move(velocidad_lineal, velocidad_angular, 2*tiempo_movimiento)

		self.tp()

	def draw_U(self):
		self.teleport(self.posicion_actual + 0.01, 6.7, -3.1416/2)

		# Vertical
		self.move(velocidad_lineal, 0.0, 0.7*tiempo_movimiento)

		# Curva
		self.move(velocidad_lineal, velocidad_angular*2, tiempo_movimiento)
		self.stop()
		# Vertical
		self.move(velocidad_lineal, 0.0, 0.7*tiempo_movimiento)

		self.tp()
		
def main(args=None):
	rclpy.init(args=args)
	nodo = NodoEscritor()
	rclpy.spin_once(nodo, timeout_sec=1.0)
	nodo.destroy_node()
	rclpy.shutdown()




