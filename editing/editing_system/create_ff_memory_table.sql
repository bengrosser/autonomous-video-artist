CREATE TABLE FF_Memory(
	memory_stamp text PRIMARY KEY, 
	related_video_name text NOT NULL, 
	ff_memory blob NOT NULL  
);
