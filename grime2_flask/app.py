from flask import Flask
from datetime import datetime
from flask import render_template

app = Flask(__name__)

@app.route("/")
def home():
    return render_template("home.html")

# New functions
@app.route("/about/")
def about():
    return render_template("about.html")

@app.route("/images/")
def images():
    return render_template("images.html")

@app.route("/contact/")
def contact():
    return render_template("contact.html")

