import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from RpiMotorLib import RpiMotorLib

def map_output(output, output_min, output_max, servo_min, servo_max):
    # Mapeia a saída do PID para o intervalo do servo motor
    output_range = output_max - output_min
    servo_range = servo_max - servo_min
    output_scaled = (output - output_min) / output_range
    servo_scaled = output_scaled * servo_range
    servo_value = servo_min + servo_scaled
    return servo_value

def main():
    # Configuração dos pinos GPIO do Raspberry Pi
    GPIO_pins_left = (0, 2, 3, 5)  # PULL+, PULL-, DIR+, DIR- do motor de passos da esquerda
    GPIO_pins_right = (23, 24, 25, 16)  # PULL+, PULL-, DIR+, DIR- do motor de passos da direita

    # Inicialização da biblioteca RpiMotorLib para os motores de passo
    mymotortest_left = RpiMotorLib.BYJMotor("Motor1", "28BYJ", GPIO_pins_left)
    mymotortest_right = RpiMotorLib.BYJMotor("Motor2", "28BYJ", GPIO_pins_right)

    #Velocidade base
    base_speed = 1350  #velocidade base constante

    #Dataframe para armazenar os dados
    df = pd.DataFrame(columns=['valores'])
    # parâmetros do controlador
    Kp = 1
    Ki = 0.1
    Kd = 0.01

    # posição e orientação inicial do robô
    x = 0

    # tempo de amostragem
    dt = 1

    # inicializa os erros e a soma acumulada de erros
    erro_x = 0
    last_error_x = 0
    soma_erros_x = 0

    # loop principal do controlador
    for i in range(1000):
        print(f"Iteração: {i}")
        desejada = 2#Posição em X(linha reta) desejada
        print(f"Posição desejada: {desejada}")

        erro_x = int(desejada) - x
        print(f"Erro: {erro_x}")

        # calcula as saídas do controlador PID discreto para a velocidade linear e angular do robô
        x = Kp * erro_x + Ki * soma_erros_x + Kd * (erro_x - last_error_x)

        #Atualizando o ultimo erro(e(t-1))
        last_error_x = erro_x

        # atualiza a soma acumulada de erros
        soma_erros_x += erro_x
        print(f"Valor de X: {x}")
        df = df.append({'valores': x}, ignore_index=True)#Ignore index atualiza o index do dataframe a cada iteração

        # Converte a saída do PID para o valor do servo motor
        servo_value = map_output(x, 0, 1730, 0, 1730)
        print(f"Valor servo: {servo_value}")

        # Ajusta a velocidade dos motores para seguir a trajetória desejada
        left_motor = base_speed - servo_value
        right_motor = base_speed + servo_value
        print(f"Motor Esquerdo: {left_motor}")
        print(f"Motor Direito: {right_motor}\n")

        # Movimentação dos motores de passo para posições desejadas baseadas no PID
        mymotortest_left.setSpeed(left_motor)#Velocidade dos motores
        mymotortest_right.setSpeed(right_motor)

        mymotortest_left.moveTo(1)#Irá se mover a uma quantidade de um único passo do motor
        mymotortest_right.moveTo(1)

        # Execução dos movimentos
        mymotortest_left.run()
        mymotortest_right.run()

    dados = df['valores']
    print(dados)
    index = np.arange(1000)
    print(index)
    plt.plot(index, dados)  # plotar o gráfico de linhas
    plt.xlabel('Index')  # adicionar rótulo no eixo x
    plt.ylabel('Cálculo do PID')  # adicionar rótulo no eixo y
    plt.title('Curva do PID')  # adicionar título ao gráfico
    plt.show()  # exibir o gráfico
    #df.to_csv('testPID.csv', index=False)
main()





