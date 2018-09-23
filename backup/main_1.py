#!/usr/bin/env python
import sys
from eventstream import *
from colorsys import *
from circularworld import *
from pointlandmark import *
from actor import *
from pacedactor import *
from camera import *
from telegraph import *
from finishline import *
from gpscamera import *
from linearprogresscamera import *
from cyclicprogresscamera import *
from autonomouscamera import *
from planner import *


def main():
  if (len(sys.argv) != 2):
    print("Usage: " + sys.argv[0] + " <config_file>")
    sys.exit(1)

  config = {}
  execfile(sys.argv[1], config)

  actors = []
  cameras = []
  landmarks = []

  theWorld = CircularWorld(actors, cameras, landmarks)  # Initialize the world

  ###########################
  # Set up the landmarks
  ###########################
  try:
    for l in config['landmarks']:
      cons = eval(l['type'])  # Turn the string 'type' into the constructor
      landmarks.append(cons(theWorld, l))
    # Check that no two of the landmarks have the same name
    for l in [x.name for x in landmarks]:
      if len([1 for y in landmarks if y.name == l]) > 1:
        raise Exception(
          "More than one landmark called \'" + l + "\'.\nThis will cause problems in interpreting events.")
      if ('(' in l) or (')' in l):
        raise Exception(
          "Landmark's can't have parentheses in their names. Please use <>, [], {} instead.\nThis will cause problems in interpreting events.")

  except KeyError:
    None  # NOP

  ###########################
  # Set up the runners
  ###########################
  try:
    for r in config['runners']:
      cons = eval(r['type'])  # Turn the string 'type' into the constructor
      actors.append(cons(theWorld, r))

    # Check that no two of actors have the same name
    for a in [x.name for x in actors]:
      if len([1 for y in actors if y.name == a]) > 1:
        raise error("More than one actor called \'" + a + "\'.\nThis will cause problems in interpreting events.")

  except KeyError:
    None  # NOP
  # Assign them jersey colors
  i = 0
  for a in actors:
    a.colr = to_int_cols(hsv_to_rgb(i / (float(len(actors))), 0.5, 0.8))
    a.colr2 = to_int_cols(hsv_to_rgb(i / (float(len(actors))), 0.5, 0.5))
    a.colr3 = to_int_cols(hsv_to_rgb(i / (float(len(actors))), 0.5, 0.1))
    i = i + 1

  ###########################
  # Set up the cameras
  ###########################
  # See which other ones are in the config file
  try:
    for c in config['cameras']:
      cons = eval(c['type'])  # Turn the string 'type' into the constructor
      newc = cons(theWorld, c)
      cameras.append(newc)
  except KeyError:
    None  # NOP

  ###########################
  # Set up the eventstream
  ###########################
  eventStream = EventStream(theWorld, c)  # Set-up a stream describing all the events

  for cam in cameras:
    cam.set_event_stream_writer(StreamWriter(eventStream))

    ###########################
  # Add the top down camera
  ###########################
  # The world always has a particular privileged camara called "godseye" that
  # the particular world knows how to produce. We add it last to that it is the
  # last one updated. This is important because it permits other cameras to move
  # during their update step, and the godseye can reflect their new position.
  cameras.append(theWorld.get_gods_eye_view())

  ###########################
  # Set up the planner
  ###########################
  thePlanner = Planner(theWorld, config, StreamReader(eventStream), cameras)

  ##############################################
  # Run the race, calling the update loop
  ##############################################
  theWorld.run_race(thePlanner)


def to_int_cols(tup):
  (r, g, b) = tup
  return (int(r * 255), int(g * 255), int(b * 255), 255)


if __name__ == '__main__':
  main()

