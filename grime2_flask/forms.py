from wtforms import Form, StringField, PasswordField, IntegerField, validators

class LoginForm(Form):
    username=StringField('Username')
    password=PasswordField('Password')
    
form = LoginForm()