from flask import Flask, request, render_template


app = Flask(__name__)


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

    return render_template('index.html', theta=theta, L=L, d=d, a=a)
