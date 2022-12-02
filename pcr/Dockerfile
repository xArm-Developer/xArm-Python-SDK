FROM python:3.8-slim-buster

WORKDIR /app

COPY . .

RUN git clone https://github.com/ultralytics/yolov5
RUN pip install --upgrade pip
RUN pip3 install -r requirements.txt

EXPOSE 7000

CMD bash