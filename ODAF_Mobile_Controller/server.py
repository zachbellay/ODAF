from aiohttp import web
import socketio
import math
import json
from decimal import Decimal
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)

# creates a new Async Socket IO Server
sio = socketio.AsyncServer()
# Creates a new Aiohttp Web Application
app = web.Application()
# Binds our Socket.IO server to our Web App
# instance
sio.attach(app)

# we can define aiohttp endpoints just as we normally
# would with no change
async def index(request):
    with open('index.html') as f:
        return web.Response(text=f.read(), content_type='text/html')

# serve javascript library that enables accelerometer access
async def gyro(request):
    with open('./dist/gyronorm.complete.min.js') as f:
        return web.Response(text=f.read(), content_type='text/javascript')
# If we wanted to create a new websocket endpoint, 
# use this decorator, passing in the name of the 
# event we wish to listen out for
@sio.on('message')
async def print_message(sid, message):
    data = json.loads(str(message).replace("\'", "\""))
    # print(data['alpha'], data['beta'], data['gamma'])
    alpha = data['alpha']
    gamma = data['gamma']
    beta = max(min(data['beta'], 90), -90) * -1


    # Limit rotation (only -30 to 30 )
    if(alpha < 30):
        alpha = alpha * -1
    elif(alpha < 360 and alpha > 330):
        alpha = 360 - alpha
    elif(alpha > 30 and alpha < 180):
        alpha = -30
    elif(alpha < 330 and alpha > 180):
        alpha = 30
    
    normalized_alpha = alpha / 30
    normalized_gamma = gamma / 90
    normalized_beta = beta / 90
    

    # Suppress noise in holding still
    if(abs(normalized_alpha) < 0.30):
        output_alpha = 0
    else:
        output_alpha = normalized_alpha

    if(abs(normalized_beta) < 0.10):
        output_beta = 0
    else:
        output_beta = normalized_beta

    if(abs(normalized_gamma) < 0.10):
        output_gamma = 0
    else:
        output_gamma = normalized_gamma

    # print('{:.4f} {:.4f} {:.4f}'.format(output_beta, output_gamma, output_alpha))
    

    r_dir=Decimal(output_alpha).quantize(Decimal('1.0000'))
    x_vec=Decimal(output_gamma).quantize(Decimal('1.0000'))
    y_vec=Decimal(output_beta).quantize(Decimal('1.0000'))
    
    output_string = str(x_vec)+","+str(y_vec)+","+str(r_dir)+"\n"
    ser.write(output_string.encode())    
    print(output_string)
    


# We bind our aiohttp endpoint to our app
# router
app.router.add_get('/', index)
app.router.add_get('/dist/gyronorm.complete.min.js', gyro)

# We kick off our server
if __name__ == '__main__':
    web.run_app(app)