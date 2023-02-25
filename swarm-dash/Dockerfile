FROM python:3.9
WORKDIR /swarm-dash/
COPY requirements.txt ./requirements.txt
RUN pip install -r requirements.txt
COPY . .
#CMD [ "gunicorn", "--workers=5", "--threads=1", "-b 0.0.0.0:80", "app:server"]
ENTRYPOINT ["streamlit", "run", "/swarm-dash/streamlit_app.py", "--server.port=8501", "--server.address=0.0.0.0"]
