from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

# Store robot position data
robot_data = {'x': [], 'y': [], 'phi': []}

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/update', methods=['POST'])
def update():
    data = request.json
    x = data.get('x')
    y = data.get('y')
    phi = data.get('phi')

    robot_data['x'].append(x)
    robot_data['y'].append(y)
    robot_data['phi'].append(phi)

    # Emit the new data to update the plot in real-time
    socketio.emit('update_plot', {'x': x, 'y': y, 'phi': phi})

    return jsonify({"status": "success"})

@socketio.on('connect')
def handle_connect():
    emit('initial_data', robot_data)

if __name__ == '__main__':
    socketio.run(app, host='192.168.43.165', port=5000)
