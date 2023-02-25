import os
import pathlib
import streamlit as st
import asyncio
import aiohttp
import time 
import plotly.graph_objs as go
import plotly.express as px
import yaml
import requests
import pandas as pd
from PIL import Image


MAP_PATH = "map.pgm"#"/swarm-dash/robohub/robohub.pgm"
MAP_YAML = "/swarm-dash/robohub/robohub.yaml"



graph_config = dict({'scrollZoom': False,'displayModeBar': False,"showAxisDragHandles": False,})

APP_PATH = str(pathlib.Path(__file__).parent.resolve())

st.set_page_config(layout="wide")

c1, c2 = st.columns([10,1])
c1.title("FireScope Dashboard")
c2.image(Image.open('assets/logo-removebg.png'))
if 'fig' not in st.session_state:
    st.session_state['fig'] = go.Figure()


if 'loop' not in st.session_state:
    st.session_state['loop'] = True

def get_sessions():
    api_url = "http://api:8000/db/readings/sessions"
    response = requests.get(api_url)
    sessions = response.json()['Sessions']
    return sessions

async def create_data(robot_num: str, mode: str, session_option: str):
    async with aiohttp.ClientSession() as session:

        api_url = f"http://api:8000/db/readings/{session_option}/{robot_num}" 
        async with session.get(api_url) as resp:
            response = await resp.json()
            fig = st.session_state.fig
            fig.data = []
            timestamps = response['Timestamps']
            Pressures = response['Pressures']
            Temperatures = response['Temperatures']
            X = response['X']
            Y = response['Y']

            map = Image.open(MAP_PATH)
            w, h = map.size

            fig = px.imshow(map, color_continuous_scale='gray')
            fig.update_traces(hovertemplate="x: %{x} <br> y: %{y}<extra></extra>")
            fig.add_trace(go.Scatter(x=X,y=Y,marker=dict(size=10,symbol="diamond", line=dict(width=2, color="DarkSlateGrey"),color=Pressures if mode=='Pressure' else Temperatures,colorscale="redor"),
            name='',customdata=Pressures if mode=='Pressure' else Temperatures,hovertemplate='<br>%{customdata:.2f} hPa' if mode=='Pressure' else '<br>%{customdata:.2f} °C', hoverlabel = dict(namelength = 50),mode='markers'))
            fig.layout.coloraxis.showscale = False

            fig.update_layout(
                # width=700,
                # height=700,
                xaxis_showgrid=False, yaxis_showgrid=False,margin=go.layout.Margin(
                    l=0, #left margin
                    r=0, #right margin
                    b=0, #bottom margin
                    t=0, #top margin
                )
            )
            fig.layout.xaxis.fixedrange = True
            fig.layout.yaxis.fixedrange = True
            fig.update_xaxes(visible=False)

            #y axis    
            fig.update_yaxes(visible=False)
            return fig, Temperatures, Pressures
    

with st.sidebar:
    session_option = st.selectbox('Choose Session',get_sessions(),index=2)
    robot_option = st.selectbox(
    'Choose Robot',
    ['1', '2', '3'],index=2)


    if st.button('Enable Thermal Camera Feed'):
        st.write('Doesnt do anything yet')

    run_map = st.checkbox("Stop Map")

    st.caption('Choose data to display on the map.')
    map_mode = st.selectbox(
        'Map Mode',
        ('Pressure', 'Temperature'))
    

st.subheader("Real-time map")
placeholder = st.empty()

async def keep_updating_placeholders(placeholder):
    while not run_map:
        _ = await asyncio.sleep(1)
        with placeholder.container():

            fig, Temperatures, Pressures = await create_data(robot_option, map_mode, session_option)
            max_temp = round(max(Temperatures),2)
            max_pres = round(max(Pressures),2)

            st.metric(label="Max Temperature", value=str(max_temp) + " °C", delta=str(max_temp) + " °C")
            st.metric(label="Max Pressure", value=str(max_pres) + " hPa", delta=str(max_pres) + " hPa")
            st.plotly_chart(fig,theme="streamlit" , **{'config': graph_config},use_container_width=True)
        
if st.session_state.loop:
    asyncio.run(keep_updating_placeholders(placeholder))




