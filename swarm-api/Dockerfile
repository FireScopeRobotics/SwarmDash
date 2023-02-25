FROM python:3.9

WORKDIR /swarm-api/

# set env variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# install dependencies
COPY requirements.txt .
RUN pip install -r requirements.txt

# copy project
COPY . .
CMD uvicorn app.main:app --proxy-headers --host 0.0.0.0 --port 8000


