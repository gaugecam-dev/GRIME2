from wtforms import StringField, PasswordField, SubmitField, RadioField
from wtforms.validators import DataRequired
from flask_wtf import Form

class LoginForm(Form):
    username=StringField('Username', [DataRequired()])
    password=PasswordField('Password', [DataRequired()])
    submit=SubmitField('Login')
    
class SettingsForm(Form):
    timestamp_source = RadioField('Timestamp source', coerce=int, choices=[(0,'EXIF metadata'),(1,'File name')])
    