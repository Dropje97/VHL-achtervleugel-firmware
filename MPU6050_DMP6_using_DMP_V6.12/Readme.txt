als je de CAN lib niet hebt gedownload kun je die uit de CAN map halen. als je de CAN lib dubbel hebt geeft dit problemen met compileren daarom staat die appart.
uncomment calibratie(); om de offset te bepalen. upload en draai de code vul daarna de waarde bij offset in en comment calibratie zodat de waarde vast staat.
de waarde van de pitch, roll en yaw hebben ongeveer 40s nodig om helemaal stabiel te worden. als de eerste 40s problemen geven kan er een delay toegevoegd worden in de setup. zie https://wired.chillibasket.com/2015/01/calibrating-mpu6050/ voor meer info
 