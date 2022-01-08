from flask import Flask, flash, render_template, redirect, request, session
from flask.helpers import url_for
from forms import LoginForm
import os

app = Flask(__name__)
app.secret_key = os.urandom(55)

@app.route("/")
def home():
    return render_template("home.html")

@app.route('/login/', methods=['post', 'get'])
def login():
    try:
        form = LoginForm(request.form)
        if request.method == 'POST':
            msg = 'user=' + form.username.data + " pass=" + form.password.data
            if form.username.data == 'admin' and form.password.data == 'password':
                flash('Login correct: ' + msg)
                return redirect(url_for('home'))
            else:
                flash('Login failed: ' + msg)
                return redirect(url_for('login'))
        return render_template('login.html', form=form)
    except Exception as e:
        return(str(e))

@app.route('/logout/')
def logout():
    session.pop('logged_in', None)
    flash('You were logged out.')
    return redirect('home')

@app.route("/images/")
def images():
    if not session.get('logged_in'):
        return redirect(url_for('login'))
    else:
        return render_template("images.html")

@app.route("/about/")
def about():
    return render_template("about.html")

@app.route("/contact/")
def contact():
    return render_template("contact.html")

