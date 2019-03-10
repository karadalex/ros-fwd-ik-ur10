#!/usr/bin/env python
from flask import Flask, request, render_template
import rospy
import json
from math import pi


app = Flask(__name__)
theta = []
L = []
d = []
a = []
joint_vars = []


@app.route('/', methods=['POST', 'GET'])
def home():
    global theta, L, d, a, joint_vars

    # Handle input from set-parameters-form
    if request.method == 'POST' and "theta" in request.form:
        theta = [float(eval(x)) for x in request.form['theta'].split(',') if x != ""]
        L = [float(eval(x)) for x in request.form['L'].split(',') if x != ""]
        d = [float(eval(x)) for x in request.form['d'].split(',') if x != ""]
        a = [float(eval(x)) for x in request.form['a'].split(',') if x != ""]
        dh = {"theta": theta, "L": L, "d": d, "a": a}
        rospy.loginfo("DH parameters: " + json.dumps(dh))
        rospy.set_param("dh_params", dh)

    # Handle input from check-joint-variables-table-form
    if request.method == 'POST' and "joint-variables" in request.form:
        joint_vars = request.form.getlist('joint-variables')
        rospy.loginfo("Joint variable names: " + json.dumps(joint_vars))
        print(joint_vars)

    return render_template('index.html', theta=theta, L=L, d=d, a=a, joint_vars=joint_vars)


if __name__ == '__main__':
    rospy.init_node('webserver')
    app.run(debug=True, host='0.0.0.0')
