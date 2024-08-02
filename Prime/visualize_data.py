from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)


robot_data = {
    'x': 0,
    'y': 0,
    'phi': 0
}

@app.route('/')
def index():
    return render_template('index.html', robot_data=robot_data)

@app.route('/update', methods=['POST'])
def update():
    global robot_data
    data = request.get_json()
    robot_data['x'] = data['x']
    robot_data['y'] = data['y']
    robot_data['phi'] = data['phi']
    
    socketio.emit('update_data', robot_data)
    
    return jsonify(success=True)

if __name__ == '__main__':
    socketio.run(app, host='192.168.43.165', port=5000)
