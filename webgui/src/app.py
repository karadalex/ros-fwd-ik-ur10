from flask import Flask, request, render_template
import rospy
import json


app = Flask(__name__)
rospy.init_node('webserver')


@app.route('/', methods=['POST', 'GET'])
def home():
    theta = []
    L = []
    d = []
    a = []
    if request.method == 'POST':
        theta = request.form['theta'].split(',')
        L = request.form['L'].split(',')
        d = request.form['d'].split(',')
        a = request.form['a'].split(',')
        dh = {"theta": theta, "L": L, "d": d, "a": a}
        rospy.loginfo("DH parameters: " + json.dumps(dh))
        rospy.set_param("dh_params", dh)

    return render_template('index.html', theta=theta, L=L, d=d, a=a)

