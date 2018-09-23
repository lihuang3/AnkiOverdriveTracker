from stream import *
from enum import Enum

from eventstream import EventType

# Because EventPrediction has two fields: eventType and other, which are
# of the same form as Events, the event_matcher in Automaton can match these as well

class EventPrediction(object):
    def __init__(self, ev, other, loc, when, pr): 
        super(EventPrediction, self).__init__()
        self.eventType = ev
        self.other = other
        self.location = loc
        self.time = when 
        self.prob = pr

    def __str__(self):
        s = str(self.time) + ": " + EventType.shortname(self.eventType)+ " " 
        for (k,v) in self.other.items():
            s = s + " " + k + ": " + " '" + v.name + "' "
        s = s + "location : " + str(self.location)
        s = s + " time : " + str(self.time)
        return s

class ActorStateType(Enum):
        WARMINGUP = 0
        RUNNING = 1
        COMPLETED = 2

class ActorPredModel(object):

    def __init__(self, actr): 
        super(ActorPredModel, self).__init__()
        self.the_actor = actr
        self.state = ActorStateType.WARMINGUP
        self.last_seen_at = -1.0;
        self.last_seen_when = -1.0;
        self.velocity_estimate = 0.1

class EventPredictor(object):

    def __init__(self, the_world):
        super(EventPredictor, self).__init__()
        self._events = []

        self._theworld = the_world

        self._landmarks = the_world.landmarks
        # We have each landmark

        self._actors = {} # dictionary of actors
        # We have model for each actor
        for a in the_world.actors:
            self._actors[a.name] = ActorPredModel(a)

        self._winner_yet = False


    def add_event(self, new_event):
        # When an event has been reported to the planner, it gives it to us
        # here so that we can use this information for future predictions
        self._events.append(new_event)
        #print("Predictor got: " + new_event.desc + "   " + str(new_event.other))
        if (new_event.eventType == EventType.STARTED):
            # Update our actor's state
            aname = new_event.other['actor'].name
            self._actors[aname].state = ActorStateType.RUNNING
            self._actors[aname].last_seen_at = 0.0
            self._actors[aname].last_seen_when = new_event.time

        elif (new_event.eventType == EventType.FINISHED):
            # Update our actor's state
            aname = new_event.other['actor'].name
            self._actors[aname].state = ActorStateType.COMPLETED
            self._actors[aname].last_seen_at = 1.0
            self._actors[aname].last_seen_when = new_event.time

            self._winner_yet = True

        elif (new_event.eventType == EventType.WINSRACE):
            # We don't do anything here, because a FINISHED event will
            # also be generated and it is handled above.
            pass

        elif (new_event.eventType == EventType.LANDMARK):
            aname = new_event.other['actor'].name
            lmark = new_event.other['landmark']

            # There is a slight correction here because the actor will move through a whole timestep, but we actually have the 
            # landmark position and the time when the actor was at the landmark. This is written in this way, because the landmark may be
            # in a different position (because the world is cyclic).
            act_loc =  self._theworld.get_lap_progress(lmark.position) - self._theworld.get_lap_progress(new_event.other['actor'].progress)
            act_loc += new_event.other['actor'].progress
            # You might ask why go to the previous effort? Mainly because the velocity estimate is pretty sparse, and it was introducing a bias,
            # as the actor was always past the landmark

            delta_x =  act_loc - self._actors[aname].last_seen_at
            delta_t = new_event.time - self._actors[aname].last_seen_when

            if (delta_t < 0): 
               # We've seen the actor since this point in time; so the estimate is outdated. we just quash things
               return
               

            #print "act_loc = ", act_loc, "      last seen at = ", self._actors[aname].last_seen_at 
            #print "delta_t: new_even.time = ",  new_event.time, "     self._actors[aname].last_seen_when = ", self._actors[aname].last_seen_when  

            assert(delta_x >= 0) # actors shouldn't go backwards
            if delta_t == 0:
                return # Probably been reported by two cameras

            if (self._actors[aname].velocity_estimate == 0):
                self._actors[aname].velocity_estimate = (delta_x / delta_t)
            else:
                self._actors[aname].velocity_estimate = 0.5 * (self._actors[aname].velocity_estimate + (delta_x / delta_t))

            #print("[" + aname + "] is with delta_t=" + str(delta_t) + " and delta_x=" +str(delta_x) + ".")

            self._actors[aname].last_seen_at = act_loc
            self._actors[aname].last_seen_when = new_event.time

        elif (new_event.eventType == EventType.OVERTAKING):

            where =  self._theworld.get_lap_progress(new_event.other['pos']) - self._theworld.get_lap_progress(new_event.other['faster'].progress)
            where += new_event.other['faster'].progress

            for a in ['faster', 'slower']:
                aname = new_event.other[a].name

                delta_x =  where - self._actors[aname].last_seen_at
                delta_t = new_event.time - self._actors[aname].last_seen_when

                if delta_x < 0: delta_x = 0

                #assert(delta_x >= 0) # actors shouldn't go backwards
                if delta_t == 0:
                    break # Probably been reported by two cameras

                if (self._actors[aname].velocity_estimate == 0):
                    self._actors[aname].velocity_estimate = (delta_x / delta_t)
                else:
                    self._actors[aname].velocity_estimate = 0.5 * (self._actors[aname].velocity_estimate + (delta_x / delta_t))

                #print("[" + aname + "] is with delta_t=" + str(delta_t) + " and delta_x=" +str(delta_x) + ".")


                #a = [x for x in self._theworld.actors if (x.name == aname)][0]
                #print a
                #p = a.progress
                #p = -1
                #print("2[" + aname + "] last seen at =" + str(self._actors[aname].last_seen_at) + " about to become =" +str(where) + " (cf. "+str(p)+")")
                self._actors[aname].last_seen_at = where
                self._actors[aname].last_seen_when = new_event.time
 
        elif (new_event.eventType == EventType.GPSINFO):
            # GPS gives ideal postioning data

            aname = new_event.other['actor'].name

            if (self._actors[aname].last_seen_at > 0):
                delta_x =  new_event.other['actor'].progress - self._actors[aname].last_seen_at
            else:
                delta_x =  0.0
            delta_t = new_event.time - self._actors[aname].last_seen_when

            if (delta_t < 0): 
               # We've seen the actor since this point in time; so the estimate is outdated. we just quash things
               return

            #if (delta_x < 0):
            #    print "aname = ", aname
            #    print "last seen at = ", self._actors[aname].last_seen_at
            #    print "other = ", new_event.other
            #    print "other[actor] = ", new_event.other['actor']
            #    print "prog = ", new_event.other['actor'].progress 
            #    print "delta_t = ", delta_t
            #    print "delta_x = ", delta_x

#            assert(delta_x >= 0) # actors shouldn't go backwards
            if delta_t == 0:
                return # Probably been reported by two cameras

            if (self._actors[aname].velocity_estimate == 0):
                self._actors[aname].velocity_estimate = (delta_x / delta_t)
            else:
                self._actors[aname].velocity_estimate = 0.5 * (self._actors[aname].velocity_estimate + (delta_x / delta_t))
                #self._actors[aname].velocity_estimate = (delta_x / delta_t)

            #print("[" + aname + "] is with delta_t=" + str(delta_t) + " and delta_x=" +str(delta_x) + " and vel="+str(self._actors[aname].velocity_estimate)+".")

            self._actors[aname].last_seen_at = new_event.other['pos']
            self._actors[aname].last_seen_when = new_event.time



        else:
            print("Unhandled event by the predictor")

    def get_predictions(self, start_time, end_time):
        # The initial design had a seperate stream for event predictions,
        # I've changed that idea. The stream seems to imply the asynchronous
        # injection of data. Instead, you get a list now, a single set of
        # predictions.
        
        # start_time -- end_time is the interval in which we're interested in hearing about predicted events
        # Note: The time difference needs to be small with respect to the velocities of the actors, otherwise
        #       bad things can happen (where, within the interval, an actor does multiple laps, for example).


        anticipated_evs = []

        if (start_time <= 0): # Predict starting for each actor who hasn't started.
            for (k, a) in self._actors.items():
                if (a.state == ActorStateType.WARMINGUP):
                    anticipated_evs.append(EventPrediction(EventType.STARTED, {'actor': a.the_actor}, 0, 0.0, 0.99))

        # Generate landmark and finish events
        for (k, a) in self._actors.items(): # For every runner 
            if (a.state == ActorStateType.RUNNING): # who is running:
                # extrapolate to where they would be at start_time
                time_to_start = start_time - a.last_seen_when
                time_to_end = end_time - a.last_seen_when
                x1 = a.last_seen_at + a.velocity_estimate*time_to_start
                x2 = a.last_seen_at + a.velocity_estimate*time_to_end
                if (x2 >= 1.0):
                    # time when crossing line anticipated?
                    when = a.last_seen_when + (1.0 - a.last_seen_at) / a.velocity_estimate
                    anticipated_evs.append(EventPrediction(EventType.FINISHED, {'actor': a.the_actor}, 1, when, 0.95))

                # Check if a landmark will be crossed:
                x1p = self._theworld.get_lap_progress(x1)
                x2p = x1p + (x2 - x1)
                for l in self._landmarks:
                    land_prog = self._theworld.get_lap_progress(l.position)
                    #print("trying a = "+str(k))
                    #print("landmark l = "+str(l.name))
                    #print("     " + str(x1p) + " <= " + str(land_prog) + " <= " + str(x2p))
                    if (x1p <= land_prog) and (land_prog <= x2p):
                        when = a.last_seen_when + time_to_start + (land_prog - x1p) / a.velocity_estimate
                #        # Unfortunately, l.position is in global progress, and we can match when we're on a different lap.
                #        if x1 == x1p:
                #            completed_laps = 0
                #        else:
                #            completed_laps = int(round(1.0/(x1 - x1p)))
                #        where = completed_laps * 1.0/self._theworld.laps + land_prog
                        where = l.position
                        anticipated_evs.append(EventPrediction(EventType.LANDMARK, {'actor': a.the_actor, 'landmark' : l}, where, when, 0.95)) # This probability should be a function of distance and variance in velocity estimates, etc.

        # If none of the actors have finished, and we're anticipating some actors finish
        # we want to make race predictions for a win:
        if not self._winner_yet:
            # none of the actors has finished
            finishing = [x for x in anticipated_evs if x.eventType == EventType.FINISHED]
            finishing.sort(key=lambda v: v.time)
            if finishing:
                #print "Looks like actors are close to finishing"
                #print [(x.time, x.other['actor'].name) for x in finishing]
                for e in finishing:
                    anticipated_evs.append(EventPrediction(EventType.WINSRACE, x.other, 1, x.time, 0.95))

        # Generate overtaking events:
        start_end_pairs = []
        for (k, a) in self._actors.items(): # For every runner 
            if (a.state == ActorStateType.RUNNING): # who is running:
                # extrapolate to where they would be at start_time
                time_to_start = start_time - a.last_seen_when
                time_to_end = end_time - a.last_seen_when
                x1 = a.last_seen_at + a.velocity_estimate*time_to_start
                x2 = a.last_seen_at + a.velocity_estimate*time_to_end
                start_end_pairs.append((x1, x2, a))


        start_end_pairs.sort(key=lambda p: p[0])
        for ii in range(len(start_end_pairs)):
            for jj in range(ii+1, len(start_end_pairs)):
                i = start_end_pairs[ii]
                j = start_end_pairs[jj]
                if (i[2].velocity_estimate > j[2].velocity_estimate):


                   ci = i[0] - i[2].velocity_estimate  * time_to_start
                   cj = j[0] - j[2].velocity_estimate  * time_to_start
                   t = (cj - ci) / (i[2].velocity_estimate - j[2].velocity_estimate)

                   #print "Potential candidate for overtaking ", i[2].the_actor.name, " & ", j[2].the_actor.name

                   if ((t >= time_to_start) and (t <= time_to_end)):
                        #print "Candidate for overtaking ", i[2].the_actor.name, " & ", j[2].the_actor.name
                        #print "times = "+ str(start_time) + " and " + str(end_time)
                        #print "i[0] = " + str(i[0]) + "  i[1] = " + str(i[1]) + "    vel = " + str(i[2].velocity_estimate)
                        #print "j[0] = " + str(j[0]) + "  j[1] = " + str(j[1]) + "    vel = " + str(j[2].velocity_estimate)
                        #print "t = " + str(t) 

                        where = (j[2].velocity_estimate * (t-time_to_start) + j[0])
                        when = start_time + t
                        e = EventPrediction(EventType.OVERTAKING, {'faster': i[2].the_actor, 'slower' : j[2].the_actor}, where, when, 0.8)
                        #print e
                        #print "where = "+str(where)
                        #print "when = "+str(when)
                        anticipated_evs.append(e) # This probability should be a function of distance and variance in velocity estimates, etc.
                
        # So far we can predict landmark crossing, starting, finishing, and overtaking

        return anticipated_evs


