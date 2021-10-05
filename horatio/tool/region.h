/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/region.h */

void hor_region_delete_last_selected (void);
void hor_region_delete_selected      (void);
void hor_region_set_function         ( void (*)(int c1, int r1, int c2, int r2,
						void *data) );
void hor_region_start                ( int c, int r, void *data );
void hor_region_moving               ( int c, int r, void *data );
void hor_region_finish               ( int c, int r, void *data );
void hor_region_cancel               ( int c, int r, void *data );
void hor_region_select               ( int c, int r, void *data );
void hor_region_clear                (void);
