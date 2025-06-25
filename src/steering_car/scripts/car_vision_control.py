#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time

class HandGestureController:
    def __init__(self):
        # --- Inicialização do ROS ---
        rospy.init_node('hand_gesture_controller', anonymous=True)

        # Publishers para as rodas e direção
        self.back_left_pub = rospy.Publisher('/car/back_left_wheel_effort_controller/command', Float64, queue_size=1)
        self.back_right_pub = rospy.Publisher('/car/back_right_wheel_effort_controller/command', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/car/front_wheels_effort_controller/command', Float64, queue_size=1)

        # Subscriber para obter a velocidade das rodas (essencial para o freio)
        rospy.Subscriber('/car/joint_states', JointState, self.joint_state_callback, queue_size=1)
        
        # --- Parâmetros de Controle (similares ao do teclado) ---
        self.effort_level = 0.3      # Potência base para aceleração/ré
        self.steering_ratio = 1.5    # Quão "forte" o carro vira
        self.braking_constant = 0.01 # Constante para o freio proporcional

        # --- Variáveis de Estado ---
        self.back_left_wheel_velocity = 0.0
        self.hand_detected = False
        
        # --- Inicialização do MediaPipe e Câmera ---
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)
        
        # Frequência do loop de controle
        self.rate = rospy.Rate(20) # 20 Hz

        # Garante que o carro pare ao desligar o nó
        rospy.on_shutdown(self.cleanup)
        
        rospy.loginfo("=== Hand Gesture Controller Ready ===")
        rospy.loginfo("Move your hand into the camera frame to control the car.")
        rospy.loginfo("Close the window or press Ctrl+C to exit.")

    def joint_state_callback(self, msg):
        """Callback para atualizar a velocidade das rodas."""
        try:
            # Usamos a velocidade da roda esquerda como referência para o freio
            idx = msg.name.index('back_left_wheel_joint')
            self.back_left_wheel_velocity = msg.velocity[idx]
        except (ValueError, IndexError) as e:
            # Acontece se o joint_state não vier como esperado
            pass

    def detect_gesture_grid(self, x, y, width, height):
        """
        Detecta o gesto com base em uma grade 3x2 na tela (sem zona morta).
        Retorna uma tupla (comando_aceleracao, comando_direcao).
        """
        # <<< MUDANÇA: Apenas uma linha divisória horizontal, no meio da tela >>>
        middle_line_y = height * 0.5
        
        # Limites horizontais (continuam os mesmos, dividindo em 3 colunas)
        left_bound = width * 0.33
        right_bound = width * 0.66

        # Zona Superior (Acelerar para frente)
        if y < middle_line_y:
            if x < left_bound:
                return "forward", "left"
            elif x > right_bound:
                return "forward", "right"
            else:
                return "forward", "center"
        
        # Zona Inferior (Dar ré)
        else:
            if x < left_bound:
                return "backward", "left"
            elif x > right_bound:
                return "backward", "right"
            else:
                return "backward", "center"

    def draw_grid(self, img, width, height):
        """Desenha a grade 3x2 na imagem para visualização."""
        # <<< MUDANÇA: Desenha apenas uma linha horizontal no meio >>>
        middle_line_y = int(height * 0.5)
        
        left_bound = int(width * 0.33)
        right_bound = int(width * 0.66)
        
        # Linha horizontal central (um pouco mais grossa para destaque)
        cv2.line(img, (0, middle_line_y), (width, middle_line_y), (0, 255, 255), 2)
        
        # Linhas verticais (sem alteração)
        cv2.line(img, (left_bound, 0), (left_bound, height), (0, 255, 255), 1)
        cv2.line(img, (right_bound, 0), (right_bound, height), (0, 255, 255), 1)
    def run(self):
        """Loop principal de controle."""
        while not rospy.is_shutdown() and self.cap.isOpened():
            success, img = self.cap.read()
            if not success:
                continue
            img = cv2.flip(img, 1)
            h, w, _ = img.shape
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = self.hands.process(img_rgb)

            throttle_cmd = "brake"
            steer_cmd = "center"    
            self.hand_detected = False

            if results.multi_hand_landmarks:
                self.hand_detected = True
                hand_lms = results.multi_hand_landmarks[0]
                self.mp_draw.draw_landmarks(img, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                
                # Usamos o pulso (landmark 0) como referência de posição
                wrist = hand_lms.landmark[0]
                cx, cy = int(wrist.x * w), int(wrist.y * h)
                
                throttle_cmd, steer_cmd = self.detect_gesture_grid(cx, cy, w, h)

            # --- Lógica de Cálculo de Esforço ---
            throttle_effort = 0.0
            steering_effort = 0.0

            # Se a mão não for detectada, aplicamos o freio proporcional
            if not self.hand_detected:
                throttle_effort = -self.braking_constant * self.back_left_wheel_velocity
                throttle_cmd = "braking (no hand)"
            else:
                # Mapeia os comandos de texto para valores de esforço
                if throttle_cmd == "forward":
                    throttle_effort = -self.effort_level
                elif throttle_cmd == "backward":
                    throttle_effort = self.effort_level
                # Se for "dead_zone", o esforço permanece 0.0
                
                if steer_cmd == "left":
                    steering_effort = self.effort_level * self.steering_ratio
                elif steer_cmd == "right":
                    steering_effort = -self.effort_level * self.steering_ratio

            # --- Publicação dos Comandos ---
            self.back_left_pub.publish(Float64(throttle_effort))
            self.back_right_pub.publish(Float64(throttle_effort))
            self.steer_pub.publish(Float64(steering_effort))

            # --- Visualização ---
            self.draw_grid(img, w, h)
            display_text = f"Throttle: {throttle_cmd} | Steer: {steer_cmd}"
            cv2.putText(img, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
            cv2.imshow("Hand Gesture Control", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            self.rate.sleep()

    def cleanup(self):
        """Função de limpeza para ser chamada no shutdown."""
        rospy.loginfo("Shutting down. Stopping the car...")
        # Envia comando de esforço zero para todas as rodas
        self.back_left_pub.publish(Float64(0.0))
        self.back_right_pub.publish(Float64(0.0))
        self.steer_pub.publish(Float64(0.0))
        
        # Libera a câmera e fecha as janelas
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        controller = HandGestureController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Garante que a limpeza seja chamada mesmo se houver um erro inesperado
        cv2.destroyAllWindows()