class Kalman:
    def __init__():
        this.QAngle = 0.0
        this.QBias = 0.0
        this.RMeasure = 0.0
        this.angle = 0.0
        this.bias = 0.0
        this.rate = 0.0
        p=[[0.0,0.0],[0.0,0.0]]
        
    def kalman():
        QAngle = 0.001
        QBias = 0.003
        RMeasure = 0.03

        angle = 0.0
        bias = 0.0

        p[0][0] = 0.0
        p[0][1] = 0.0
        p[1][0] = 0.0
        p[1][1] = 0.0

    def getAngle(newAngle, newRate,dt):
        rate = newRate - bias;
        angle += dt * rate;

        p[0][0] += dt * (dt*p[1][1] - p[0][1] -p[1][0] + QAngle)
        p[0][1] -= dt * p[1][1]
        p[1][0] -= dt * p[1][1]
        p[1][1] += QBias * dt

        s = p[0][0] + RMeasure
        k[0] = p[0][0]/s
        k[1] = p[1][0]/s

        y = newAngle - angle

        angle += k[0] * y
        bias  += k[1] * y

        p00Temp = p[0][0]
        p01Temp = p[0][1]

        P[0][0] -= K[0] * P00Temp;
        P[0][1] -= K[0] * P01Temp;
        P[1][0] -= K[1] * P00Temp;
        P[1][1] -= K[1] * P01Temp;

        return Angle

    def setAngle(angle):
        this.angle = angle

    def setQAngle(QAngle):
        this.QAngle = QAngle

    def setQBias(QBias):
        this.QBias = QBias

    def setRMeasure(RMeasure):
        this.RMeasure = RMeasure

    def getRate():
        return this.rate

    def getQAngle():
        return this.QAngle

    def getQBias():
        return QBias

    def  getRMeasure():
        return this.RMeasure
