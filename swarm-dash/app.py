import os
import pathlib

import dash
import cv2
import rospy
from multiprocessing import Process, Manager
from nav_msgs.msg import OccupancyGrid
from dash import dcc
from dash import html
from dash.dependencies import Input, Output, State
import dash_bootstrap_components as dbc
from dash import dash_table
import dash_daq as daq
import numpy as np
import plotly.graph_objs as go
import plotly.express as px
import requests
import pandas as pd
from camera_opencv import Camera
from flask import Flask, Response



FA = "https://use.fontawesome.com/releases/v5.12.1/css/all.css"
dbc_css = "https://cdn.jsdelivr.net/gh/AnnMarieW/dash-bootstrap-templates/dbc.min.css"
external_stylesheets = [dbc.themes.CYBORG, dbc_css]
map_dict = {}


server = Flask(__name__)
app = dash.Dash(__name__, external_stylesheets=external_stylesheets,server=server)
app.title = "FireScope Robotics Dashboard"
app.config["suppress_callback_exceptions"] = True

def gen(camera):
    """Video streaming generator function."""
    yield b'--frame\r\n'
    while True:
        frame = camera.get_frame()
        yield b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n--frame\r\n'

@server.route('/video_feed')
def video_feed():
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


def map_callback(msg):
    """Load received global costmap"""
    map_dict['origin'] = [msg.info.origin.position.x, msg.info.origin.position.y]
    map_dict['height'] = msg.info.height
    map_dict['width'] = msg.info.width
    map_dict['map_res'] = msg.info.resolution
    map_dict['occupancy_map'] = np.reshape(msg.data, (map_dict['height'], map_dict['width']))
    # Convert costmap message from y-by-x to x-by-y coordinates as it is row-major order, with (0,0) at bottom left
    map_dict['occupancy_map'] = np.transpose(map_dict['occupancy_map'])
    map_dict['occupancy_map'][map_dict['occupancy_map'] == -1] = 205
    map_dict['occupancy_map'][map_dict['occupancy_map'] == 0] = 254

graph_config = {
    "displayModeBar": False,
    "scrollZoom": False,
    "showAxisDragHandles": False,
}



def get_sessions():
    api_url = "http://localhost:8000/db/readings/sessions"
    response = requests.get(api_url)
    sessions = response.json()['Sessions']
    return sessions

controls = dbc.Card(
    [   
        html.Div(
            [
                dbc.Label("Choose session"),
                dcc.Dropdown(get_sessions(), get_sessions()[0], id='session-choice',clearable=False),
            ]
        ),
        html.Div(
            [
                dbc.Label("Choose a robot"),
                dcc.Dropdown(['1', '2', '3'], '1', id='robot-choice', clearable=False),
            ]
        ),
        html.Div(
            [   
                dbc.Label("Choose map type"),
                dcc.Dropdown(['Pressure', 'Temperature'], 'Pressure', id='map-choice', clearable=False),
            ]
        ),
        html.Hr(),
    ],
    body=True,
)
max_temp_card = dbc.Card(
    children=[
        dbc.CardHeader(
            "Max Temperature [°C]",
            style={
                "display": "inline-block",
                "text-align": "center",
                "color": "white",
            },
        ),
        dbc.CardBody(
            [
                html.Div(
                    daq.Gauge(
                        id="max-temp-gauge",
                        min=0,
                        max=300,
                        showCurrentValue=True,
                        color="#a4a7db",
                        style={
                            "align": "center",
                            "display": "flex",
                            "marginTop": "5%",
                            "marginBottom": "-10%",
                        },
                    ),
                    className="m-auto",
                    style={
                        "display": "flex",
                        "border-radius": "1px",
                        "border-width": "5px",
                    },
                )
            ],
            className="d-flex",
        ),
    ],
    outline=True,
    color="light", inverse=True,
    style={"height": "85%"},
)

max_pressure_card = dbc.Card(
    children=[
        dbc.CardHeader(
            "Max Pressure [hPa]",
            style={
                "display": "inline-block",
                "text-align": "center",
                "color": "white",
            },
        ),
        dbc.CardBody(
            [
                html.Div(
                    daq.Gauge(
                        id="max-pressure-gauge",
                        min=0,
                        max=2500,
                        showCurrentValue=True,
                        color="#a4a7db",
                        style={
                            "align": "center",
                            "display": "flex",
                            "marginTop": "5%",
                            "marginBottom": "-10%",
                        },
                    ),
                    className="m-auto",
                    style={
                        "display": "flex",
                        "border-radius": "1px",
                        "border-width": "5px",
                    },
                )
            ],
            className="d-flex",
        ),
    ],
    outline=True,
    color="light", inverse=True,
    style={"height": "85%"},
)

main_child = dbc.Container(
    [        
        dbc.Row(
            [
                dbc.Col([dcc.Graph(id='real-time-map',config=graph_config), 
                        dcc.Interval(
                                id='interval-component',
                                interval=5*1000, # in milliseconds
                                n_intervals=0
                            )
                         ], md=8,lg=8),
                dbc.Col([
                        dbc.Row(
                            max_temp_card,
                        ), 
                        dbc.Row(
                            max_pressure_card,
                            style={
                                "marginTop": "5%"
                            },

                        justify='end',
                        ), 
                ], md=4,lg=4),
            ],  
            align="center",
        ),
    ],
)

CONTENT_STYLE = {
    "left": 22,
    'margin-left': '22rem',
    'margin-right': '5rem',
    'padding': '1rem' '1rem',
}

SIDEBAR_STYLE = {
    "position": "fixed",
    "top": 0,
    "left": 0,
    "bottom": 0,
    "width": "20rem",
    "padding": "2rem 1rem",
    "background-color": "#1e1d24",
}

thermal_card = dbc.Card(
    [
        dbc.CardBody(html.P("Thermal Feed", className="card-text")),
        dbc.CardImg(src="/video_feed", bottom=True),
    ],
    style={"width": "18rem"},color="light"
)

sidebar = html.Div(
    [
        html.H3("FireScope Dashboard", className="fw-bold text-wrap"),
        controls,
        html .Hr(),
        thermal_card
    ],
    style=SIDEBAR_STYLE,
)
main_page = html.Div(id='main-content', children=[main_child],style=CONTENT_STYLE)

content = html.Div(id='page-content', children=[sidebar,main_page])

app.layout = html.Div([
    content
])

def x_2_pixel(x):
    """Convert a coordinate in map frame (meters) to pixels"""
    return round((x - map_dict['origin'][0])/(map_dict['map_res']))

def y_2_pixel(y):
    """Convert a coordinate in map frame (meters) to pixels"""
    return round((y - map_dict['origin'][1])/(map_dict['map_res'])) 

def create_data(robot_num: str, mode: str, session_option: str):
    api_url = f"http://localhost:8000/db/readings/{session_option}/{robot_num}" 
    response = requests.get(api_url)
    fig = go.Figure()

    timestamps = response.json()['Timestamps']
    Pressures = response.json()['Pressures']
    Temperatures = response.json()['Temperatures']
    X = response.json()['X']
    Y = response.json()['Y']
    #Tranpose fucks it up
    Y_actual = list(map(x_2_pixel, X))# response.json()['X']
    X_actual = list(map(y_2_pixel, Y))# response.json()['X']

    map_ = map_dict['occupancy_map']


    fig = px.imshow(map_, color_continuous_scale='gray')
    fig.update_traces(hovertemplate="x: %{x} <br> y: %{y}<extra></extra>")
    fig.add_trace(go.Scatter(x=X_actual,y=Y_actual,marker=dict(size=12,symbol="circle", line=dict(width=2, color="DarkSlateGrey"),color=Pressures if mode=='Pressure' else Temperatures,colorscale="redor"),
    name='',customdata=Pressures if mode=='Pressure' else Temperatures,hovertemplate='<br>%{customdata:.2f} hPa' if mode=='Pressure' else '<br>%{customdata:.2f} °C', hoverlabel = dict(namelength = 50),mode='markers'))
    fig.layout.coloraxis.showscale = False

    fig.update_layout(
        width=900,
        height=900,
        xaxis_showgrid=False, yaxis_showgrid=False,margin=go.layout.Margin(
            l=0, #left margin
            r=100, #right margin
            b=0, #bottom margin
            t=0, #top margin
        )
    )

    fig.update_layout(
    paper_bgcolor='rgba(0,0,0,0)',
    plot_bgcolor='rgba(0,0,0,0)'
    )

    fig.layout.xaxis.fixedrange = True
    fig.layout.yaxis.fixedrange = True
    fig.update_xaxes(visible=False) 
    fig.update_yaxes(visible=False)
    return fig, Temperatures, Pressures
    


@app.callback(
    Output('real-time-map', 'figure'),
    Output('max-pressure-gauge', 'value'),
    Output('max-temp-gauge', 'value'),
    Input('robot-choice', 'value'),
    Input('map-choice', 'value'),
    Input('session-choice', 'value'),
    Input('interval-component', 'n_intervals'))
def update_figure(robot,mode, session, n_intervals):

    
    fig, Temperatures, Pressures = create_data(robot,mode,session)

    fig.update_layout(transition_duration=500)

    max_temp = max(Temperatures)
    max_pres = max(Pressures)

    return fig, max_pres, max_temp

@app.callback(
     Output('session-choice', 'options'),
     Input('interval-component', 'n_intervals'))
def update_sessions(n_intervals):
     sessions = get_sessions()

     return sessionse

# Running the server
if __name__ == "__main__":    
    rospy.init_node('dash_listener')
    rospy.Subscriber('/map_merge_topic',OccupancyGrid,map_callback)
    app.run_server(host="0.0.0.0", port=8080, debug=True,threaded=True)