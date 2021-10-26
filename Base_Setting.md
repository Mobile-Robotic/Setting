# 톱니바퀴
> General -> Frequency 14745600 설정

<img src="https://user-images.githubusercontent.com/69342532/133555852-158e8c32-428f-4c97-834a-203c6564e45c.png" width = 400>

> libraries -> liba.a & libm.a 추가

<img src="https://user-images.githubusercontent.com/69342532/133555854-bab38f74-822b-49be-948d-439d51c9f215.png" width = 500>

***
# Interface.c
<img src = "https://user-images.githubusercontent.com/69342532/133033266-3eaab591-18bb-4f00-aab4-f3139cf7ca01.png" width=600>

> Interface_init()에 putchar1(18); 추가 (카메라 V1 프로토콜 사용)
> 
<img src = "https://user-images.githubusercontent.com/69342532/133033288-e77b31bb-6db0-43cb-8489-57771b8af22e.png" width=300>

> Camera Baud Rate Setting : (USART1 Baud Rate: 115200)(통신속도)(UBRR1L=0x5F -> UBRR1L=0x07)

<img src = "https://user-images.githubusercontent.com/69342532/133033275-5abc3b02-c066-467c-921e-1b5576233698.png" width=400>

> ISR Camera_V1 Setting   

<img src = "https://user-images.githubusercontent.com/69342532/133033280-59156de6-1f55-4a80-8486-406f2765ef09.png" width=500>   

> lcd_clear_screen()을 cls()로 display_char()을 dc()로
> 
<img src = "https://user-images.githubusercontent.com/69342532/133033286-919a9467-aa17-49ab-ae38-0a26290b6731.PNG" width=200>   

***

# Interface.h
<img src = "https://user-images.githubusercontent.com/69342532/133033338-df3f3e84-bc94-4417-858c-362a4efe200c.png" width=700>
<img src = "https://user-images.githubusercontent.com/69342532/133033335-e1d43001-44bd-441a-ac70-74e9fa886a25.PNG" width=700>

***
# Motor.c

> SetGain 10, 3, 1 Setting

<img src = "https://user-images.githubusercontent.com/69342532/133034409-0301c4b1-0230-4c71-b129-46ce8158a43d.png" width=400>
<img src = "https://user-images.githubusercontent.com/69342532/133034406-74088034-507b-4090-9d84-e38afba20bf7.png" width=400>

***

# Move.c
<img src = "https://user-images.githubusercontent.com/69342532/133034629-8987ec05-c2fb-4ec1-9f2f-c4bcd51a7d64.png" width=400>
<img src = "https://user-images.githubusercontent.com/69342532/133034619-145ce12a-2898-4109-b8ae-e14878f7bb7d.png" width=400>
<img src = "https://user-images.githubusercontent.com/69342532/133034624-70372910-1ca0-4fd3-b43a-4d30cba84cb3.png" width=400>

***

# Move.h
<img src="https://user-images.githubusercontent.com/69342532/133034837-7c789634-b6e9-4674-96dc-15fb84b12392.PNG" width=800>
<img src="https://user-images.githubusercontent.com/69342532/133034828-94f1c92a-9fd5-414b-850f-8bf158422b4b.PNG" width=700>
<img src="https://user-images.githubusercontent.com/69342532/133034834-db4adbbd-a230-42f5-aa75-2691ec77a016.PNG" width=900>

***

# Sensor.h
> S에 | (~(PIND<<1)&0x20) | (~PIND&0x40) 추가
> 
<img src="https://user-images.githubusercontent.com/69342532/133034988-ff664aea-3bf9-437d-bc29-c45453b94fc6.png" width=900>

***
