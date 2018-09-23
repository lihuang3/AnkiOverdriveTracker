#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 20 18:05:49 2018

@author: lhuang28
"""
import cv2, numpy as np
from scipy import spatial
from tracker import *

class detection_module():

    def __init__(self, vid_file, skel_file, track_file):

        
        self.vid = vid_file
        self.track_mask = cv2.imread(track_file, cv2.IMREAD_GRAYSCALE);
    

        self.skel = cv2.imread(skel_file,cv2.IMREAD_GRAYSCALE)
        self.width = self.skel.shape[1]
        self.height = self.skel.shape[0]
        self.skel[0,:] = 0
        self.skel[:,0] = 0
        self.skel[self.height-1,:] = 0
        self.skel[:,self.width-1] = 0



        # Specify pixel coord for startline, finishline, \
        # and the gap (a pixel separates startline and finishline)

        # RaceV03 - track3
        # self.start = [470, 97]
        # self.finish = [472, 97]
        # self.sfgap = [471,97]

        # RaceV04- track4
        # self.start = [480, 110]
        # self.finish = [482, 110]
        # self.sfgap = [481,110]

#        # RaceV05 - track5
#        self.start = [310, 69]
#        self.finish = [312, 69]
#        self.sfgap = [311,69]
        
        # RaceV05 - track6
        self.start = [490, 103]
        self.finish = [492, 103]
        self.sfgap = [491,103]        
        self.newskel = np.zeros([1020,1463]).astype(np.uint8)

        self.coord_mapping()
        # cv2.imshow('s',self.pixMap)


        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))

        self.fgbg = cv2.BackgroundSubtractorMOG2()
        
    def coord_mapping(self):

        y,x = np.where(self.skel>0)

        self.tree = spatial.KDTree(list(zip(x,y)))

        loop_coord = np.linspace(0.0,1.0,len(y))
        self.tree_len = len(loop_coord)
        self.pixMap = np.copy(self.skel).astype(np.float32)

        self.pixMap[self.pixMap<100.] = 0.;
        self.pixMap[self.sfgap[1], self.sfgap[0]] = 0.
        self.pixMap[self.start[1],self.start[0]] = 1./(len(y)-1)
        self.pixMap[self.finish[1], self.finish[0]] = 1.0

        self.coordMap = np.zeros([self.tree_len,2], dtype = np.int32)
        self.coordMap[0,:] = self.sfgap[:]
        self.coordMap[1,:] = self.start[:]
        self.coordMap[-1,:] = self.finish[:]
        frontier = self.start
        i = 1
        while frontier !=self.finish:
            tar_row, tar_col, min_dist = -1,-1,10

            patch = np.copy( self.pixMap[frontier[1]-1:frontier[1]+2,frontier[0]-1:frontier[0]+2])

            rows, cols  = np.where(patch>100)
            if len(rows)>0:
                i +=1
                for row, col in zip(rows, cols):
                    dist = abs(row-1)+abs(col-1)
                    if dist<min_dist and dist>0:
                        tar_row, tar_col, min_dist = row,col, dist
                frontier[1] += tar_row-1
                frontier[0] += tar_col-1
                self.pixMap[frontier[1],frontier[0]] = loop_coord[i]
                self.coordMap[i,:] = frontier[:]

            else:
                break

        # Mark points out of the centerline
        self.pixMap[self.pixMap>=1.] = 1.


#        cv2.imshow('pixMap',np.asarray(self.pixMap*255., np.uint8))
#        cv2.waitKey(0)
#        cv2.destroyAllWindows()

        print i+2
        print len(loop_coord)
#        assert(i+2==len(loop_coord))

    def pix2coord(self, x, y):
        return self.pixMap[y, x]

    def coord2pix(self, coord):
        return self.coordMap[int(round(coord*(self.tree_len-1)))]

 
    def process_frame(self, frame):
#   test frame to get starting line coordinate

    

        frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#        frameGray[self.track_mask==0] = 0

        thresh1 = 40
        thresh2 = 80
        frameGray[frameGray<thresh1] = 0
        frameGray[frameGray>thresh2] = 255
        


        fgmask = self.fgbg.apply(frameGray)
        kernel = np.ones((10,10),np.uint8)
        kernel1 = np.ones((9,9),np.uint8)
        fgmask = cv2.erode(fgmask, kernel, iterations = 1)
        fgmask = cv2.dilate(fgmask,kernel1,iterations = 1)
        
        # fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, self.kernel)
        
        self.fgmask = np.copy(fgmask)
        contours, _ = cv2.findContours(fgmask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        positions = []
        if len(contours)>0:
            mean_area = 0
            tuples = []
            for contour in contours:
                cnt = contour
                area = cv2.contourArea(cnt)
                x,y,w,h = cv2.boundingRect(cnt)
                
                # rect = cv2.minAreaRect(cnt)
                # box = cv2.boxPoints(rect)
                # box = np.int0(box)

                if area>100 and area<2000:
                    # cv2.drawContours(frame,[box],0,(0,0,255),2)
                    mean_area+=area
                    tuples.append([x,y,w,h,area])
                    
            new_tuples = np.reshape(tuples,[-1,5])
            if len(tuples)>4:          
                mean_area /=4.
                tuples = np.reshape(tuples, [-1,5])
                res = np.where(tuples[:,4]<0.667*mean_area)[0]
                
                if np.shape(res)[0] == 2:
                    tl = np.amin(tuples[res,:2],axis=0)
                    br = np.amax(tuples[res,:2]+tuples[res,2:4],axis=0)     
                    x,y = tl
                    w,h = br-tl                    
                    ccat_area = w*h
                    
                    if ccat_area<2*mean_area and ccat_area>0.33*mean_area:
                        
                        new_tuples = tuples[tuples[:,4]>=0.667*mean_area]
                        new_tuples = np.append(new_tuples, [[x,y,w,h,ccat_area]],axis=0)            
                elif np.shape(res)[0] == 1:
                    bbox = tuples[res[0],:4]
                    ext_bbox = np.copy(bbox)
                    ext_bbox[:2] = bbox[:2]+0.5*bbox[2:4]-bbox[2:4]
                    ext_bbox[2:4] *=2
                    new_tuples = tuples[tuples[:,4]>=0.667*mean_area]
                    candidates = new_tuples[:,:4]
                    iom_arr = iom(ext_bbox, candidates)
                    max_match = np.amax(iom_arr)

                    if max_match>=0.25:
                        index = np.where(iom_arr == max_match)
                        index = np.squeeze(index)
                        bbox2 = candidates[index,:]
                        tmp = np.reshape(np.append(bbox,bbox2),[-1,4])
                        tl = np.amin(tmp[:,:2],axis=0)
                        br = np.amax(tmp[:,:2]+tmp[:,2:4],axis=0)     
                        x,y = tl
                        w,h = br-tl                    
                        ccat_area = w*h                        
                        if ccat_area<2*mean_area and ccat_area>0.33*mean_area:
                            new_tuples[index,:] = np.array([x,y,w,h,ccat_area])
                        
                        
            for i in range(np.shape(new_tuples)[0]):         
                x,y,w,h,area = new_tuples[i].astype(np.int16)
                dist, idx = self.tree.query([x,y])
                tx,ty = self.tree.data[idx]
                coord = self.pix2coord(tx,ty)
                positions.append([coord,x,y,w,h])



        return positions
    

class camera_module():
    def __init__(self, detection_module, num_cams):
        self.progress = np.zeros([num_cams])
        self.coord = self.progress
        self.num_cams = num_cams
        self.goal_coord = self.coord
        self.ROI_w = 200
        self.ROI_h = 200     
        self.det_obj = detection_module
        self.cam_stream = []
        for i in range(self.num_cams):
            self.cam_stream.append([])

    def get_ROI(self, cam):
        x, y = self.det_obj.coord2pix(self.coord[cam])
        x1, y1 = x-self.ROI_w/2,y-self.ROI_h/2
        x2, y2 = x+self.ROI_w/2,y+self.ROI_h/2
        ROI = [x1,y1,x2,y2]
        return ROI

    def draw_ROI(self,frame):
        for cam in range(self.num_cams):
            x1, y1, x2, y2 = self.get_ROI(cam)
            cv2.rectangle(frame, ( int(x1), int(y1) ),\
                ( int(x2), int(y2)),(0,255,0),2)
        return frame

    def get_coord(self):
        return self.coord

    def set_goal(self, cam, destination):
        self.goal_coord[cam] = destination*13.-np.floor(destination*13.)

    def move(self, cam, destination):
#        while destination>1.:
#            destination-=1.
        self.coord[cam] = destination*13.-np.floor(destination*13.)

        # dist = self.goal_coord-self.coord
        # self.coord += np.sign(dist)*np.amin( zip([self.ds]*self.num_cams, abs(np.asarray(dist))), axis = 1)
        # np.amin(  zip([ds]*3,abs(np.asarray(dist))), axis= 1)
#        for cam in range(self.num_cams):
#            x1,y1,x2,y2 = self.get_ROI(cam)
#            patch = frame[y1:y2,x1:x2]
#           #  self.cam_stream[cam].append(cv2.cvtColor(patch, cv2.COLOR_BGR2RGB))
#            cv2.imshow('cam_%d'%(cam), patch)
#            cv2.waitKey(1)
#
#        if draw_cam:
#            frame = self.draw_ROI(frame)
