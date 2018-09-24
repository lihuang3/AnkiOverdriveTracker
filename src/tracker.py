#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 20 18:03:37 2018

@author: lhuang28
"""
import numpy as np

class Tracker():
    def __init__(self):
        self.history = []
        self.tracks = []
        self.next_id = 0
        self.max_age = 12
                
    def update(self, positions):
              
        candidates_s = np.reshape(positions, [-1,5])

        for track in self.tracks:            
#            print "track bbox %s"%(str(track.bbox))
            candidates_s = track.update(candidates_s) 
            
        for i in range(np.shape(candidates_s)[0]):
            self.init_track(candidates_s[i,:])
              
        for track in self.tracks:
            if track.state == 0 and track.time_since_update>0:
                self.tracks.remove(track)
            elif track.time_since_update>self.max_age:                
                self.history.append(track)
                self.tracks.remove(track)

    
    def init_track(self, bbox_s):
        self.next_id += 1        
        track = Track(bbox_s, self.next_id)                        
        self.tracks.append(track)    
    
        
def iom(bbox, candidates):

    bbox_tl, bbox_br = bbox[:2], bbox[:2] + bbox[2:]
    candidates_tl = candidates[:, :2]
    candidates_br = candidates[:, :2] + candidates[:, 2:]

    tl = np.c_[np.maximum(bbox_tl[0], candidates_tl[:, 0])[:, np.newaxis],
               np.maximum(bbox_tl[1], candidates_tl[:, 1])[:, np.newaxis]]
    br = np.c_[np.minimum(bbox_br[0], candidates_br[:, 0])[:, np.newaxis],
               np.minimum(bbox_br[1], candidates_br[:, 1])[:, np.newaxis]]
    wh = np.maximum(0., br - tl)

    area_intersection = wh.prod(axis=1)
    area_bbox = bbox[2:].prod()
    area_candidates = candidates[:, 2:].prod(axis=1)
    _iou = area_intersection / (area_bbox + area_candidates - area_intersection)
    _iom = area_intersection / np.minimum(area_bbox, area_candidates)

    return _iou
    
class Track():
    def __init__(self, bbox_s, Id):        
        self.id = Id
        self.state = 0
        self.bbox = bbox_s[1:]
        self.bbox_pred = bbox_s[1:]
        self.hits = 1
        self.time_since_update = 0
        self.ninit = 1
        self.lap = 0
        self.coord = bbox_s[0]
        self.dt = 1./30.
        self.ux = 0.
        self.uy = 0.
        self.vel = 0.0
        self.max_vel = 0.
        self.progress = 0.0
        
    def update(self, candidates_s):
        candidates = candidates_s[:,1:]
        coords = candidates_s[:,0]
        if np.shape(candidates)[0]>0:
            self.bbox[:2] +=[self.ux,self.uy]
            self.bbox_pred = self.bbox
            iom_arr = iom(self.bbox, candidates)
            max_match = np.amax(iom_arr)
            if max_match>=0.25:
                index = np.where(iom_arr == max_match)
                index = np.squeeze(index)
                self.ux,self.uy = 0.6*np.append(self.ux,self.uy) \
                        +0.4*(candidates[index,:2]+0.5*candidates[index,2:]-self.bbox[:2]-0.5*self.bbox[2:])
                self.vel = np.sqrt(np.square(self.ux)+np.square(self.ux))
                if self.max_vel<self.vel:
                    self.max_vel = self.vel
                self.bbox = candidates[index,:]
                
                if coords[index]<self.coord and abs(self.ux)>=4:
                    self.lap+=1
                self.coord = coords[index]    
                self.progress = (self.lap+self.coord)/13.
                if self.progress<5e-3/13.0:
                    self.progress = 0.0
                if self.progress>=1.:
                    self.progress = 1.0
                candidates_s = np.delete(candidates_s, index, 0)
    
                self.time_since_update = 0
                self.hits += 1                                  
                
                if self.state == 0 and self.hits>=self.ninit:
                    self.state = 1                            
            else:
                self.time_since_update+=1
        else:
            self.time_since_update+=1

        return candidates_s