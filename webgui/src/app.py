#!/usr/bin/env python
from flask import Flask, request, render_template
import rospy
import json
from math import pi


app = Flask(__name__)


@app.route('/', methods=['POST', 'GET'])
def home():
    theta = []
    L = []
    d = []
    a = []
    if request.method == 'POST':
        theta = [float(eval(x)) for x in request.form['theta'].split(',') if x != ""]
        L = [float(eval(x)) for x in request.form['L'].split(',') if x != ""]
        d = [float(eval(x)) for x in request.form['d'].split(',') if x != ""]
        a = [float(eval(x)) for x in request.form['a'].split(',') if x != ""]
        dh = {"theta": theta, "L": L, "d": d, "a": a}
        rospy.loginfo("DH parameters: " + json.dumps(dh))
        rospy.set_param("dh_params", dh)

    return render_template('index.html', theta=theta, L=L, d=d, a=a)


if __name__ == '__main__':
    rospy.init_node('webserver')
    app.run(debug=True, host='0.0.0.0')
