#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import ApplyJointEffort
from std_srvs.srv import Empty
from rospy.exceptions import ROSInterruptException
from pynput import keyboard

keys_pressed = set()
EFFORT_AMOUNT = 3.0  # Define the default effort amount

def apply_effort(joint_name, effort, duration_secs):
    """Aplica um esforço a uma junta específica no Gazebo"""
    rospy.wait_for_service('/gazebo/apply_joint_effort')
   
    try:
        apply_effort_srv = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        start_time = rospy.Time(0)
        duration = rospy.Duration(duration_secs)
        resp = apply_effort_srv(joint_name, effort, start_time, duration)
        if resp.success:
            rospy.loginfo(f"Effort {effort} applied to {joint_name}")
        else:
            rospy.logwarn(f"Failed to apply effort to {joint_name}: {resp.status_message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def apply_effort_to_wheels(effort, duration_secs=1.0):
    """Aplica o mesmo esforço para ambas as rodas"""
    apply_effort("front_left_wheel_joint", effort, duration_secs)
    apply_effort("front_right_wheel_joint", effort, duration_secs)

def on_press(key):
    """Callback quando uma tecla é pressionada"""
    global EFFORT_AMOUNT  # Declare the variable as global to modify it
    
    try:
        if key == keyboard.Key.up and key not in keys_pressed:
            rospy.loginfo(f"Tecla 'seta para cima' pressionada. Aplicando effort {EFFORT_AMOUNT}")
            apply_effort_to_wheels(EFFORT_AMOUNT)
            keys_pressed.add(key)
            
        elif key == keyboard.Key.down and key not in keys_pressed:
            rospy.loginfo(f"Tecla 'seta para baixo' pressionada. Aplicando effort {-EFFORT_AMOUNT}")
            apply_effort_to_wheels(-EFFORT_AMOUNT)
            keys_pressed.add(key)
            
        elif hasattr(key, 'char') and key.char == '+' and key not in keys_pressed:
            EFFORT_AMOUNT += 1
            rospy.loginfo(f"Tecla '+' pressionada. AUMENTANDO PARA {EFFORT_AMOUNT}")
            keys_pressed.add(key)
        
        elif hasattr(key, 'char') and key.char == '-' and key not in keys_pressed:
            EFFORT_AMOUNT -= 1
            rospy.loginfo(f"Tecla '-' pressionada. DIMINUINDO PARA {EFFORT_AMOUNT}")
            keys_pressed.add(key) 
            
    except Exception as e:
        rospy.logerr(f"Erro ao processar tecla pressionada: {e}")

def on_release(key):
    """Callback quando uma tecla é solta"""
    try:
        if key == keyboard.Key.up or key == keyboard.Key.down:
            rospy.loginfo("Tecla de seta solta. Parando (effort 0)")
            apply_effort_to_wheels(0.0)
            keys_pressed.discard(key)
            
        elif hasattr(key, 'char') and (key.char == '+' or key.char == '-'):
            keys_pressed.discard(key)
            
        if key == keyboard.Key.esc:
            # Parar o listener
            return False
            
    except Exception as e:
        rospy.logerr(f"Erro ao processar tecla solta: {e}")

def main():
    rospy.init_node('keyboard_effort_controller', anonymous=True)
    
    rospy.loginfo("🚗 Controlador de Teclado Iniciado")
    rospy.loginfo("Use a seta para cima/baixo para mover o robô")
    rospy.loginfo("Use + e - para ajustar a intensidade do esforço")
    rospy.loginfo("Pressione ESC para sair")
    
    # Iniciar a captura de teclas em um thread não-bloqueante
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    # Manter o nó ROS rodando
    try:
        rospy.spin()
    except ROSInterruptException:
        pass
    finally:
        # Garantir que paramos o robô ao finalizar
        apply_effort_to_wheels(0.0)
        listener.stop()

if __name__ == '__main__':
    main()