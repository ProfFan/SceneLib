/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/cor_file.h */

#ifdef _HORATIO_IMAGE_
Hor_Bool        hor_write_corner_map ( Hor_Corner_Map *,
				       const char *base_name );
Hor_Corner_Map *hor_read_corner_map  ( const char *base_name );
#endif /* _HORATIO_IMAGE_ */
