from wtforms import StringField, PasswordField, SubmitField
from wtforms.validators import DataRequired
from flask_wtf import Form

class LoginForm(Form):
    username=StringField('Username', [DataRequired()])
    password=PasswordField('Password', [DataRequired()])
    submit=SubmitField('Login')
    