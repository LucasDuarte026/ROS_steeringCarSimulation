import cv2
import mediapipe as mp
import rospy
from std_msgs.msg import Float64

# Inicializa o ROS
rospy.init_node('hand_gesture_controller')
back_left_pub = rospy.Publisher('/car/back_left_wheel_effort_controller/command', Float64, queue_size=10)
back_right_pub = rospy.Publisher('/car/back_right_wheel_effort_controller/command', Float64, queue_size=10)
steer_pub = rospy.Publisher('/car/front_wheels_effort_controller/command', Float64, queue_size=10)

# Inicializa o MediaPipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

# Inicializa a webcam
cap = cv2.VideoCapture(0)

# Parâmetros similares ao controle por teclado
effort_level = 0.1
steering_ratio = 0.5

# Função para detectar gesto com base na posição da mão
def detect_gesture(x, y, width, height):
    if y < height * 0.3:
        return "forward"
    elif y > height * 0.7:
        return "stop"
    elif x < width * 0.3:
        return "left"
    elif x > width * 0.7:
        return "right"
    else:
        return "wait"

# Variável para evitar publicar o mesmo gesto repetidamente
last_gesture = ""

while not rospy.is_shutdown():
    success, img = cap.read()
    if not success:
        continue

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)

    h, w, _ = img.shape
    gesture = "wait"

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(img, handLms, mp_hands.HAND_CONNECTIONS)
            wrist = handLms.landmark[0]
            cx, cy = int(wrist.x * w), int(wrist.y * h)
            gesture = detect_gesture(cx, cy, w, h)
            break  # só uma mão

    cv2.putText(img, f"Gesture: {gesture}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)

    if gesture != last_gesture and gesture != "wait":
        # Mapear gesto para esforço das rodas e direção
        if gesture == "forward":
            left_effort = right_effort = -effort_level
            steering_effort = 0.0
        elif gesture == "stop":
            left_effort = right_effort = 0.0
            steering_effort = 0.0
        elif gesture == "left":
            left_effort = right_effort = 0.0
            steering_effort = effort_level * steering_ratio
        elif gesture == "right":
            left_effort = right_effort = 0.0
            steering_effort = -effort_level * steering_ratio
        else:
            left_effort = right_effort = steering_effort = 0.0

        back_left_pub.publish(Float64(left_effort))
        back_right_pub.publish(Float64(right_effort))
        steer_pub.publish(Float64(steering_effort))

        rospy.loginfo(f"Publicado gesto: {gesture} | Esforço rodas: {left_effort:.2f}, {right_effort:.2f} | Direção: {steering_effort:.2f}")
        last_gesture = gesture

    cv2.imshow("Hand Gesture Control", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
