/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/image_io.h */

#ifdef HOR_TRANSPUTER
#ifdef _channel_h

void       HorChanInImageHeader       (Channel *channel, Hor_Image     *image);
void       HorChanInSubImageHeader    (Channel *channel, Hor_Sub_Image *image);
void       HorChanInImageData         (Channel *channel, Hor_Image     *image);
Hor_Image     *HorChanInImage         (Channel *channel);
Hor_Sub_Image *HorChanInSubImage      (Channel *channel);
void       HorChanInAllocatedImage    (Channel *channel, Hor_Image     *image);
void       HorChanInAllocatedSubImage (Channel *channel, Hor_Sub_Image *image);
void       HorChanOutImageHeader      (Channel *channel, Hor_Image     *image);
void       HorChanOutSubImageHeader   (Channel *channel, Hor_Sub_Image *image);
void       HorChanOutImageData        (Channel *channel, Hor_Image     *image);
void       HorChanOutImage            (Channel *channel, Hor_Image     *image);
void       HorChanOutSubImage         (Channel *channel, Hor_Sub_Image *image);

#define HorChanInSubImageData( channel, im ) \
           HorChanInImageData ( channel, &(im)->image )
#define HorChanOutSubImageData( channel, im ) \
           HorChanOutImageData ( channel, &(im)->image )

#endif
#endif
