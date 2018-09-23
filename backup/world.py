import pdb
from tracker import *
from detection import *
import cv2, sys, os, av
import numpy as np
from scipy import spatial


class World(object):
    """A class for managing the state of the marathon environment"""
    # The class manages details of whether this is a linear track, or a 
    # circular one, or whatever.

    sim_delta_t = 0.1
    gods_eye_camera = None # To be setup in an overridden constructor to initialize it to something more meaninful

    def __init__(self, actors, cams, landmarks):
        self.sim_time = 0;
        self.actors = actors
        self.cams = cams 
        self.landmarks = landmarks

    def actor_finished(self, actor):
        return actor.progress >= 1.0

    def get_embedding(self, progress):
        return None # To be overridden appropriately

    def get_lap_progress(self, progress):
        return progress # To be overridden in worlds with more than 1 lap

    def race_over(self):
        for a in self.actors:
            if not self.actor_finished(a):
                    return False;
        return True; 

    def update_cams(self):
        for c in self.cams:
            c.update(self, self.actors)


    def run_race(self, planner):
        
        
        color_dict = {0:(0,0, 255), 1:(0,255,255), 2:(255, 0,0), 3:(148, 0, 211), 4: (0, 127, 255),
                5:(0,255,0), 6:(130, 0, 75), 7:(255,255,255), 8:(0,0,0)}
        
        # Set up actor information
        actor_dict = {1: 'Alice', 2: 'Bob', 3: 'Candice', 4: 'Dave'}
#        cam_dict = {'AC0':0,'AC1':1,'AC2':2,'Start':3}
        
        # Set cam era information
        cam_dict = {'AC0':0,'AC2':1,'Start':2}
        
        # Number of cameras
        num_cams = 3
        
        # Load experiment data
        vid_file = av.open('./experiment/RaceCar.flv')
        skel_file = './experiment/skel.png'
        track_file = './experiment/track.png'
        
    
        det_obj = detection_module(vid_file, skel_file, track_file)
        tracker = Tracker()
        cam_obj = camera_module(det_obj, num_cams=num_cams)

        # Set up fpor output video
        container = av.open('./experiment/output.mp4', mode='w')
        container.metadata['title'] = 'test_output'
        container.metadata['key'] = 'value'
        fps = 29
        vid_stream = container.add_stream('mpeg4', rate=fps)
        vid_stream.width = int(955 * 1.5)
        vid_stream.height = int(680 * 1.5)
        vid_stream.pix_fmt = 'yuv420p'
        

        planner.start()

        # Give the cameras some time for their establishing shot:
        for c in self.cams:
            c.opening_shot(self, self.actors)
        
        # Take all the shots
        self.update_cams()
        planner.step()

        
        actor_trajectory = [[],[],[],[]]
        cam_trajectory = [[],[],[]]
        vel_profile = [[],[],[],[]]
        
        # Video frame counter        
        num_frames = 0
        
        for frame_obj in det_obj.vid.decode(video=0):
            if self.race_over():
                break
 
            num_frames +=1
            print '\rframe %d' % (num_frames),
            sys.stdout.flush()
            
            frame = np.array(frame_obj.to_image())
            # Crop frame
            frame = frame[45:1065, 225:1688, :].astype(np.uint8)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Obtain actor positions
            positions = det_obj.process_frame(frame)

            ext_positions = []

            # Extend bbox area by 3 times for better tracking
            if len(positions) > 0:
                for item in positions:
                    coord, x, y, w, h = item
                    ext_positions.append([coord, x - 1.5 * w, y - 1.5 * h, 4 * w, 4 * h])
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)

            # Update tracker
            tracker.update(ext_positions)

            # Draw actor bbox and print progress information
            for track in tracker.tracks:
                x, y, w, h = track.bbox.astype(np.int16)

                if track.state == 1 and track.time_since_update > 1:
                    continue
                cv2.putText(frame, str(actor_dict[track.id]), (int(x + 0.625 * w + 10), int(y + 0.5 * h)), 0,
                            5e-3 * 200, (255, 255, 255), 2)
                cv2.putText(frame,
                            str(actor_dict[track.id]) + ', lap ' + str('%.3f ' % (track.lap+track.coord)),
                            (400, 400 + int(track.id * 45)), 0, 5e-3 * 200, (255, 255, 255), 2)

            # Race cars appear at this frame! Start the simulator
            if num_frames>=176:
                self.sim_time = self.sim_time + World.sim_delta_t
                actor_progress_dict ={}
                for track in tracker.tracks:
                    actor_progress_dict[actor_dict[track.id]] = track.progress
                    actor_trajectory[track.id-1].extend([track.progress])
                    vel_profile[track.id-1].extend([track.vel])

                for a in self.actors:
                    # pdb.set_trace()
                    a.step(self.sim_time)
                    a.progress = actor_progress_dict[a.name]

                
                self.update_cams()
                for c in self.cams:
                    if c.name in cam_dict:
                        cam_obj.move(int(cam_dict[c.name]),c.progress)
                        cam_trajectory[int(cam_dict[c.name])].extend([c.progress])
                        x1,y1,x2,y2 = cam_obj.get_ROI(int(cam_dict[c.name]))
                        cv2.rectangle(frame, (int(x1), int(y1)), \
                                      (int(x2), int(y2)), (0, 255, 0), 2)

                planner.step()

            # Draw frame information
#                cv2.putText(frame, str(num_frames), (int(50), int(300)), 0, 5e-3 * 300, (255, 255, 255), 3)
            cv2.line(frame, (470,0), (470,220), color_dict[(2)%9],  3) 
            
            # Save frames 
#            cv2.imwrite('./experiment/frames/f_%04d.png'%(num_frames), frame)
            
            # Save the output video
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            frame_out = av.VideoFrame.from_ndarray(frame, format='rgb24')

            packet = vid_stream.encode(frame_out)

            if packet is not None:
                container.mux(packet)

        planner.finish()
        
        # Save racing information
        for track in tracker.tracks:
            print 'Actor %d max vel = %.2f'%(track.id, track.max_vel)
        
        with open('./experiment/actor_trajectory.txt', 'w') as f:
            for traj in actor_trajectory:
                for item in traj:
                    f.write('%.4f '%(item)) 
                f.write('\n')
        
        with open('./experiment/vel_profile.txt', 'w') as f:
            for traj in vel_profile:
                for item in traj:
                    f.write('%.2f '%(item)) 
                f.write('\n')        
        with open('./experiment/cam_trajectory.txt', 'w') as f:
            for traj in cam_trajectory:
                for item in traj:
                    f.write('%.4f '%(item)) 
                f.write('\n')
 
        
        
            # Give the cameras some time for their closing shot:
        for c in self.cams:
            c.closing_shot(self, self.actors)  # Finish encoding the stream

        for packet in vid_stream.encode():
            container.mux(packet)

        # Close the file
        container.close()

    def get_gods_eye_view(self):
            # This returns the special privileged camera that knows about the structure of
            # the world to draw the actors as well as potentially the other cameras
            return self.gods_eye_camera



