/*
 * Catmull-Rom splines
 *
 *
 * Mariano Abad (weimaraner@gmail.com)
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#define HERMITESPLINE 1
#define LERPSPLINE 0
#define CATMULLSPLINE 2
#define SMOOTHSPLINE 3
uint8_t  SPLINETYPE = LERPSPLINE;

SDL_Window* window = NULL;
SDL_Renderer* renderer = NULL;
int mousePosX , mousePosY ;
int xnew , ynew ;
int32_t controlPoints[9];
uint8_t posCounter;
uint32_t frames=0;
int32_t X_OFFSET =100;
int32_t Y_OFFSET =300;
double SplineTension =0;

static double double_map(double valueCoord1,
                         double startCoord1, double endCoord1,
                         double startCoord2, double endCoord2) {
    double offset = startCoord2;
    double ratio = (endCoord2 - startCoord2) / (endCoord1 - startCoord1);
    return ratio * (valueCoord1 - startCoord1) + offset;
}

/*Function to draw all other 7 pixels present at symmetric position*/
void drawCircle(int xc, int yc, int x, int y)
{
    SDL_RenderDrawPoint(renderer,xc+x,yc+y) ;
    SDL_RenderDrawPoint(renderer,xc-x,yc+y);
    SDL_RenderDrawPoint(renderer,xc+x,yc-y);
    SDL_RenderDrawPoint(renderer,xc-x,yc-y);
    SDL_RenderDrawPoint(renderer,xc+y,yc+x);
    SDL_RenderDrawPoint(renderer,xc-y,yc+x);
    SDL_RenderDrawPoint(renderer,xc+y,yc-x);
    SDL_RenderDrawPoint(renderer,xc-y,yc-x);
}

/*Function for circle-generation using Bresenham's algorithm */
void circleBres(int xc, int yc, int r)
{
    int x = 0, y = r;
    int d = 3 - 2 * r;
    while (y >= x)
    {
        /*for each pixel we will draw all eight pixels */
        drawCircle(xc, yc, x, y);
        x++;

        /*check for decision parameter and correspondingly update d, x, y*/
        if (d > 0)
        {
            y--;
            d = d + 4 * (x - y) + 10;
        }
        else
            d = d + 4 * x + 6;
        drawCircle(xc, yc, x, y);
    }
}

void drawAxis() {
    int32_t x;

    SDL_SetRenderDrawColor(renderer, 32, 32, 32, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLine(renderer, 100, 50, 100, 650);
    SDL_RenderDrawLine(renderer, 50, 300, 1000, 300);

    for (x = 0 + X_OFFSET; x < ((4096 * 2) / 10) + X_OFFSET; x = x + 10) {
        SDL_RenderDrawLine(renderer, x, Y_OFFSET - 5, x, Y_OFFSET + 5);

    }

    for (x = 0 + X_OFFSET; x < ((4096 * 2) / 10) + X_OFFSET; x = x + 10) {
        SDL_RenderDrawLine(renderer, x, Y_OFFSET - 5, x, Y_OFFSET + 5);

    }
}


double Clamp(double value, double min, double max)
{
    // First we check to see if we're greater than the max
    value = (value > max) ? max : value;

    // Then we check to see if we're less than the min.
    value = (value < min) ? min : value;

    // There's no check to see if min > max.
    return value;
}
/*
   Tension: 1 is high, 0 normal, -1 is low
   Bias: 0 is even,
         positive is towards first segment,
         negative towards the other
*/
double HermiteInterpolate(
        double y0,double y1,
        double y2,double y3,
        double mu,
        double tension,
        double bias)
{
    double m0,m1,mu2,mu3;
    double a0,a1,a2,a3;

    mu2 = mu * mu;
    mu3 = mu2 * mu;
    m0  = (y1-y0)*(1+bias)*(1-tension)/2;
    m0 += (y2-y1)*(1-bias)*(1-tension)/2;
    m1  = (y2-y1)*(1+bias)*(1-tension)/2;
    m1 += (y3-y2)*(1-bias)*(1-tension)/2;
    a0 =  2*mu3 - 3*mu2 + 1;
    a1 =    mu3 - 2*mu2 + mu;
    a2 =    mu3 -   mu2;
    a3 = -2*mu3 + 3*mu2;

    return(a0*y1+a1*m0+a2*m1+a3*y2);
}



double CatmullRom(double value1, double value2, double value3, double value4, double amount)
{
    // Using formula from http://www.mvps.org/directx/articles/catmull/
    double amountSquared = amount * amount;
    double amountCubed = amountSquared * amount;
    return (double)(0.5 * (2.0 * value2 +
                           (value3 - value1) * amount +
                           (2.0 * value1 - 5.0 * value2 + 4.0 * value3 - value4) * amountSquared +
                           (3.0 * value2 - value1 - 3.0 * value3 + value4) * amountCubed));
}
double Hermite(double value1, double tangent1, double value2, double tangent2, double amount)
{
    double v1 = value1, v2 = value2, t1 = tangent1, t2 = tangent2, s = amount, result;
    double sCubed = s * s * s;
    double sSquared = s * s;

    if (amount == 0)
        result = value1;
    else if (amount == 1)
        result = value2;
    else
        result = (2 * v1 - 2 * v2 + t2 + t1) * sCubed +
                 (3 * v2 - 3 * v1 - 2 * t1 - t2) * sSquared +
                 t1 * s +
                 v1;
    return (double)result;
}

double Lerp(double value1, double value2, double amount)
{
    return value1 + (value2 - value1) * amount;
}


double SmoothStep(double value1, double value2, double amount)
{
    // It is expected that 0 < amount < 1
    // If amount < 0, return value1
    // If amount > 1, return value2

    double result = Clamp(amount, 0, 1);
    result = Hermite(value1, 0, value2, 0, result);
    return result;
}
/* fillPositionsSpline
 * totalFrames: total frames of the movement
 * controlPoints: array of control points
 * controlPointsNb: number of controlPoints
 * splineType:  0: linear
 *              1: hermite
 *              2: catmull
 *              3: smoothstep
 *
 * tension: -1 .. 1  ( 0.5 centripedal )
 * bias : -1 .. 1   (0)
 * framesArr: filled frames positions array
 * maxFrames: len of frames array
*/
uint8_t fillPositionsSpline(uint16_t totalFrames, int32_t * controlPoints, uint16_t controlPointsNb, uint8_t type,double tension , double bias, int32_t * framesArr, uint16_t maxFrames){
    int32_t myControlPoints[12];
    uint16_t myControlPointsNb;
    int32_t i;
    int32_t j;
    double framesT;

    if ( maxFrames < totalFrames){
        return 10; // error totalframes are less then maxFrames
    }

    framesT = (double)totalFrames / (controlPointsNb-1);

    myControlPointsNb = controlPointsNb + 2;
    myControlPoints[0] = myControlPoints[1];


    for ( i=1 ; i <= controlPointsNb ; i++ ){
        myControlPoints[i] = controlPoints[ i - 1 ];
    }

    myControlPoints[ controlPointsNb + 1 ] = controlPoints[ controlPointsNb -1 ];

    // so far , we got a new array of position with the dummy start and end knots of the curve

    for ( i = 0 ;  i < controlPointsNb - 1 ; i++ ){

        for ( j = ((double)totalFrames/(controlPointsNb-1))*i; j<((double)totalFrames/(controlPointsNb-1))*(i+1) ;j++ ) {
            int32_t p0,p1,p2,p3;
            int32_t Y,X;
            double t;

            t = ((double) 1 / ((double)totalFrames / (posCounter - 1))) * (j - (((double)totalFrames / (posCounter - 1)) * i));

            p0 = myControlPoints[i];
            p1 = myControlPoints[i+1];
            p2 = myControlPoints[i+2];
            p3 = myControlPoints[i+3];
            if (type == 0)
                Y = Lerp(p1,p2,t);
            if (type == 1)
                Y = HermiteInterpolate( p0, p1, p2, p3, t, tension, bias);
            else if (type ==2 )
                Y = CatmullRom(p0,p1,p2,p3,t);
            else if ( type ==3 )
                Y = SmoothStep(p1,p2,t);
            X = j;
            framesArr[X]=Y;

        }
    }

    return 0; // OK
}

void drawPoints(int32_t * framesArr, uint16_t totalFrames,uint8_t connected){
    double x_mapped,y_mapped;
    double x_mapped2,y_mapped2;

    for ( int i =0 ; i <totalFrames;i++){
        x_mapped = double_map(i,0,frames,0,800) ;//X_OFFSET
        y_mapped = double_map(framesArr[i],-300000,300000,300,-300);
       // printf("Frame: %.3d - Position: %.6d\r\n",i,framesArr[i]);

        SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderDrawPoint(renderer , (int)x_mapped+X_OFFSET, (int)y_mapped+Y_OFFSET) ;
    }

    if ( connected){

        for (int i = 0; i <= totalFrames - 1; i++) {


            x_mapped = double_map(i , 0, frames, 0, 800);//X_OFFSET
            y_mapped = double_map(framesArr[i], -300000, 300000, 300, -300);
            x_mapped2 = double_map(i+1, 0, frames, 0, 800);//X_OFFSET
            y_mapped2 = double_map(framesArr[i + 1], -300000, 300000, 300, -300);

            SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);
            SDL_RenderDrawLine(renderer, (int) x_mapped + X_OFFSET, (int) y_mapped + Y_OFFSET,
                               (int) x_mapped2 + X_OFFSET, (int) y_mapped2 + Y_OFFSET);


        }
    }
    return;
}

void fillVelocity(int32_t * framesArr,double *velArray,uint16_t totalFrames){

    for ( int i =1 ; i <=totalFrames;i++){

        if ( i <= totalFrames -1){
            velArray[i] = (double) (framesArr[i+1] - framesArr[i] );
        }else{
            velArray[i] = 0;
        }
    }
    return;
}

void fillAccel(double *velArray,double *accelArray,uint16_t totalFrames){

    for ( int i =1 ; i <=totalFrames;i++){

        if ( i <= totalFrames -1){
            accelArray[i] = (double) (velArray[i+1] - velArray[i] );
        }else {
            accelArray[i] = 0;
        }
    }
    return;
}

void drawVelocity(double * veloArray, uint16_t totalFrames,uint8_t connected){
    double x_mapped,y_mapped;
    double x_mapped2,y_mapped2;

    for ( int i =0 ; i <=totalFrames;i++){


        x_mapped = double_map(i-1,0,frames,0,800) ;//X_OFFSET
        y_mapped = double_map(veloArray[i],-50000,50000,300,-300);

        SDL_SetRenderDrawColor(renderer, 0, 255, 255, SDL_ALPHA_OPAQUE);
        SDL_RenderDrawPoint(renderer , (int)x_mapped+X_OFFSET, (int)y_mapped+Y_OFFSET) ;



    }
    if (connected){
        for (int i = 1; i <= totalFrames - 1; i++) {


            x_mapped = double_map(i - 1, 0, frames, 0, 800);//X_OFFSET
            y_mapped = double_map(veloArray[i], -50000, 50000, 300, -300);
            x_mapped2 = double_map(i, 0, frames, 0, 800);//X_OFFSET
            y_mapped2 = double_map(veloArray[i + 1], -50000, 50000, 300, -300);

            SDL_SetRenderDrawColor(renderer, 0, 255, 255, SDL_ALPHA_OPAQUE);
            SDL_RenderDrawLine(renderer, (int) x_mapped + X_OFFSET, (int) y_mapped + Y_OFFSET,
                               (int) x_mapped2 + X_OFFSET, (int) y_mapped2 + Y_OFFSET);


        }
    }
    return;
}


void drawAccel(double *accelArray,uint16_t totalFrames,uint8_t connected){
    double x_mapped,y_mapped,velo,accel;
    double x_mapped2,y_mapped2;
    for ( int i =1 ; i <=totalFrames;i++){


        x_mapped = double_map(i-1,0,frames,0,800) ;//X_OFFSET
        y_mapped = double_map(accelArray[i],-5000,5000,300,-300);

        SDL_SetRenderDrawColor(renderer, 255, 255, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderDrawPoint(renderer , (int)x_mapped+X_OFFSET, (int)y_mapped+Y_OFFSET) ;



    }
    if(connected) {


        for (int i = 1; i <= totalFrames - 1; i++) {


            x_mapped = double_map(i - 1, 0, frames, 0, 800);//X_OFFSET
            y_mapped = double_map(accelArray[i], -5000, 5000, 300, -300);
            x_mapped2 = double_map(i, 0, frames, 0, 800);//X_OFFSET
            y_mapped2 = double_map(accelArray[i + 1], -5000, 5000, 300, -300);

            SDL_SetRenderDrawColor(renderer, 255, 255, 0, SDL_ALPHA_OPAQUE);
            SDL_RenderDrawLine(renderer, (int) x_mapped + X_OFFSET, (int) y_mapped + Y_OFFSET,
                               (int) x_mapped2 + X_OFFSET, (int) y_mapped2 + Y_OFFSET);


        }
    }

    return;
}

void drawControlPoints(int32_t* controlPoints,uint16_t controlPointsNb,int16_t totalFrames) {
    double p,x_mapped,y_mapped;

    for (int  i = 0 ; i < controlPointsNb ; i++){

        p = ((double)totalFrames/(controlPointsNb-1))*i;
        x_mapped = double_map(p,0,totalFrames,0,800) ;//X_OFFSET
        y_mapped = double_map(controlPoints[i],-300000,300000,300,-300);

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderDrawPoint(renderer , (int)x_mapped+X_OFFSET, (int)y_mapped+Y_OFFSET) ;
        circleBres( (int)x_mapped+X_OFFSET,(int)y_mapped+Y_OFFSET,5);

    }
}

void drawBanner(void){

    TTF_Init();
    TTF_Font* Sans = TTF_OpenFont("../FreeSans.ttf", 18);
    if(!Sans) {
        printf("TTF_OpenFont: %s\n", TTF_GetError());
        exit(0);
    }
    SDL_Color White = {255, 255, 255};
    SDL_Surface* surfaceMessage = TTF_RenderText_Blended_Wrapped(Sans," l: Linear \r\n h: Hermite\r\n c: Catmull\r\n s: SmoothStep\r\n r: Randomize\r\n t: -0.1 tension\r\n y: +0.1 tension\r\n q: quit", White,150);
    SDL_Texture* Message = SDL_CreateTextureFromSurface(renderer, surfaceMessage);
    SDL_Rect Message_rect;

    Message_rect.x = 0;
    Message_rect.y = 500;
    Message_rect.w = surfaceMessage->w;
    Message_rect.h = surfaceMessage->h;

    SDL_RenderCopy(renderer, Message, NULL, &Message_rect);
    SDL_DestroyTexture( Message );
    SDL_FreeSurface( surfaceMessage );
    TTF_CloseFont(Sans);
}


void drawStatus(void){
    char buffer[255];

    switch (SPLINETYPE){
        case 0: //lerp
            snprintf(buffer,255,"LERP");
            break;
        case 1: //hermite
            snprintf(buffer,255,"HERMITE T: %.1f",SplineTension);
            break;
        case 2: //catmull
            snprintf(buffer,255,"CATMULL-ROM");
            break;
        case 3: //smooth
            snprintf(buffer,255,"SMOOTHSTEP");
            break;
    }
    TTF_Init();
    TTF_Font* Sans = TTF_OpenFont("../FreeSans.ttf", 18);
    if(!Sans) {
        printf("TTF_OpenFont: %s\n", TTF_GetError());
        exit(0);
    }
    SDL_Color White = {255, 255, 255};

    SDL_Surface* surfaceMessage = TTF_RenderText_Blended_Wrapped(Sans,buffer, White,300);
    SDL_Texture* Message = SDL_CreateTextureFromSurface(renderer, surfaceMessage);
    SDL_Rect Message_rect;

    Message_rect.x = 0;
    Message_rect.y = 0;
    Message_rect.w = surfaceMessage->w;
    Message_rect.h = surfaceMessage->h;

    SDL_RenderCopy(renderer, Message, NULL, &Message_rect);
    SDL_DestroyTexture( Message );
    SDL_FreeSurface( surfaceMessage );
    TTF_CloseFont(Sans);
}


void drawPointStatus(uint16_t x,int16_t frames,int32_t * framesArr,double* velArray, double* accelArray){
    char buffer[255];
    uint16_t mappedx;
    mappedx = (uint16_t)double_map(x,0,800,0,frames) ;//X_OFFSET

    snprintf(buffer,255," Frame: %d \r\n Pos: %d \r\n Vel: %.2f\r\n Acc: %.2f\r\n",x,framesArr[mappedx],velArray[mappedx],accelArray[mappedx]);

    TTF_Init();
    TTF_Font* Sans = TTF_OpenFont("../FreeSans.ttf", 18);
    if(!Sans) {
        printf("TTF_OpenFont: %s\n", TTF_GetError());
        exit(0);
    }
    SDL_Color White = {255, 255, 255};

    SDL_Surface* surfaceMessage = TTF_RenderText_Blended_Wrapped(Sans,buffer, White,300);
    SDL_Texture* Message = SDL_CreateTextureFromSurface(renderer, surfaceMessage);
    SDL_Rect Message_rect;

    Message_rect.x = 900;
    Message_rect.y = 0;
    Message_rect.w = surfaceMessage->w;
    Message_rect.h = surfaceMessage->h;

    SDL_RenderCopy(renderer, Message, NULL, &Message_rect);
    SDL_DestroyTexture( Message );
    SDL_FreeSurface( surfaceMessage );
    TTF_CloseFont(Sans);
}
int main(int argc, char* argv[])
{
    int32_t framesArray[4096];
    double velArray[4096];
    double accelArray[4096];
    uint8_t connected=0;

    uint8_t contFlag = 1;
    int i;

    frames = 240;

    posCounter = 4;
    controlPoints[0] = 0;
    controlPoints[1] = -100000;
    controlPoints[2] = 200000;
    controlPoints[3] = -100000;

    /*initialize sdl*/
    if (SDL_Init(SDL_INIT_EVERYTHING) == 0)
    {
        if(SDL_CreateWindowAndRenderer(1200, 700, 0, &window, &renderer) == 0)
        {
            SDL_bool done = SDL_FALSE;

            int i = 0 ;
            int x[4] , y[4] , flagDrawn = 0 ;

            while (!done)
            {
                SDL_Event event;

                /*set background color to black*/
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
                SDL_RenderClear(renderer);

                /*set draw color to white*/
                SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
                if (SDL_WaitEvent(&event)){
                    if (event.type == SDL_QUIT)
                    {
                        done = SDL_TRUE;
                    }
                    if(event.type == SDL_MOUSEBUTTONDOWN)
                    {
                        contFlag=1;
                    }

                    if (event.type == SDL_KEYDOWN){
                        switch (event.key.keysym.sym)
                        {
                            case SDLK_l:   SPLINETYPE = 0; break;
                            case SDLK_c:   SPLINETYPE = 2; break;
                            case SDLK_h:   SPLINETYPE = 1; break;
                            case SDLK_s:   SPLINETYPE = 3; break;
                            case SDLK_t:
                                SplineTension = SplineTension -0.1;
                                break;
                            case SDLK_y:
                                SplineTension = SplineTension +0.1;
                                break;
                            case SDLK_q:
                                SDL_Quit();
                                exit(0);
                                break;
                            case SDLK_r:
                                posCounter = (rand() % (9 - 2 + 1)) + 2;
                                for (i = 0; i < posCounter; i++) {
                                    controlPoints[i] = (rand() % (300000 - -300000 + 1)) + -300000;
                                    //controlPoints[i] = (rand() % (300000 + 1));

                                }
                                break;
                            case SDLK_v:
                                connected = !connected;
                                break;
                        }
                        contFlag=1;

                    }
                    if(event.type == SDL_MOUSEMOTION)
                    {
                        /*get x and y positions from motion of mouse*/
                        xnew = event.motion.x ;
                        ynew = event.motion.y ;

                        int j ;
                        if (((xnew -X_OFFSET)>= (0) )&&((xnew -X_OFFSET)<= (800) )) {
                            drawPointStatus(xnew-X_OFFSET,frames,framesArray,velArray,accelArray);

                        }

                        /*updating mouse positions to positions
                        coming from motion*/
                        mousePosX = xnew ;
                        mousePosY = ynew ;
                        SDL_SetRenderDrawColor(renderer, 64, 64, 64, SDL_ALPHA_OPAQUE);
                        SDL_RenderDrawLine(renderer, xnew, 50, xnew, 650);
                        contFlag=1;

                    }
                }
                if ( contFlag==1 ) {

                    drawAxis();
                    //printf ( "fillPositionsSpline = %d\r\n",
                    fillPositionsSpline(frames, controlPoints, posCounter, SPLINETYPE , SplineTension , 0 ,framesArray, 4095);
                            //);
                    drawPoints(framesArray,frames,connected);
                    drawControlPoints(controlPoints,posCounter, frames) ;
                    fillVelocity(framesArray,velArray,frames);
                    fillAccel(velArray,accelArray,frames);
                    drawVelocity(velArray,frames,connected);

                    drawAccel(accelArray,frames,connected);

                    drawBanner();
                    drawStatus();

                    /*show the window*/
                    SDL_RenderPresent(renderer);
                    contFlag = 0;
                }
            }
        }

        /*Destroy the renderer and window*/
        if (renderer)
        {
            SDL_DestroyRenderer(renderer);
        }
        if (window)
        {
            SDL_DestroyWindow(window);
        }
    }
    /*clean up SDL*/
    SDL_Quit();
    return 0;
}
