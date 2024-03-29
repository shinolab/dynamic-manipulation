
#include "glRenderer.h"
#include "math.h"

#ifndef PI
#define PI 3.1415926535897f
#endif

// app globals for 3d text display
extern unsigned long NPFrameSignature;
extern unsigned long NPStaleFrames;
extern double NPLatencyAverage;
extern char szVersion[MAX_PATH];
extern char szStatus[MAX_PATH];
extern double dSampleDeltaAverage;


///////////////////////////////////////////////////////////////////////////////
// glObject
///////////////////////////////////////////////////////////////////////////////

glObject::glObject(void)
{
	Projection = Perspective;
	Color[0] = Color[1] = Color[2] = Color[3] = 1.0f;
	Position[0] = Position[1] = Position[2] = 0.0f;
	Rotation[0] = Rotation[1] = Rotation[2] = 0.0f;
	fScale = 1.0f;
	RenderStyle = 0;
	m_bVisible = true;
	bWantMouse = false;
	bWantKeyboard = false;
}


///////////////////////////////////////////////////////////////////////////////
// glSphere
///////////////////////////////////////////////////////////////////////////////

glSphere::glSphere(void)
{
    m_Radius = 5.0f;
    m_deltaLong = 5.0f;
    m_deltaLat = 5.0f;
    m_hTile = m_vTile = 1.0f;
    bShowSphereGrid = true;
    m_fPanoRotation = 0.0f;
    quadratic = gluNewQuadric();
}

glSphere::~glSphere(void)
{
    gluDeleteQuadric(quadratic);
}

void glSphere::_drawCircle(float fCenterX, float fCenterY, float fRadius, int nSections, bool bFilled)
{
   int i;
   GLfloat twoPi = 2.0f * 3.14159f;

   float matColor[4];
   matcpy(matColor, matYellowClear);
   matColor[3] = .01f;

   if(bFilled)
   {
       glBegin(GL_TRIANGLE_FAN);
       glVertex2f(fCenterX, fCenterY); 
       glColor4fv(matColor);
       for(i = 0; i <= nSections;i++)
       {
           glVertex2f(fCenterX + (fRadius * cos(i *  twoPi / nSections)), fCenterY + (fRadius* sin(i * twoPi / nSections)));
       }
       glEnd();
   }
   else
   {
       glBegin(GL_LINE_LOOP);
       for(i = 0; i <= nSections;i++)
       {
           glVertex2f(fCenterX + (fRadius * cos(i *  twoPi / nSections)), fCenterY + (fRadius* sin(i * twoPi / nSections)));
       }
       glEnd();
   }

}

void glSphere::Render(void* pClientData)
{

	glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushMatrix();

    glDisable(GL_CULL_FACE);

    // radius determines the "size" of our 3d world.  our goals is meters,  user is about 50 centimeters away from monitor,
    // effective range is about 200 centimeters (6ft), so make a volume 200 cms cubed
    float radius = 200.0f / 2.0f;
    float fFontSize = 2.0f;
    int nLats = (int) (360 / (m_deltaLat*2));
    int nLongs = (int) (360 / m_deltaLong);
    glRenderer* pRenderer = (glRenderer*)pClientData;

    Position[0] = (float) -pRenderer->tirData.x;
    Position[1] = (float) pRenderer->tirData.y;
    Position[2] = (float) pRenderer->tirData.z;
    Rotation[0] = (float) pRenderer->tirData.pitch;
    Rotation[1] = (float) pRenderer->tirData.yaw;
    Rotation[2] = (float) pRenderer->tirData.roll;


    bool bWireframe;
    if(pRenderer->m_bPano)
        bWireframe = false;
    else
        bWireframe = true;

    // draw sphere
    glPushMatrix();
    glRotatef(m_fPanoRotation, 0.0f, 1.0f, 0.0f);
    if(bWireframe)
    {
	    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	    glColor4fv(matGrey);
    }
    else
    {
	    glPolygonMode(GL_BACK, GL_FILL);
	    glPolygonMode(GL_FRONT, GL_LINE);
	    glEnable(GL_TEXTURE_2D);						        
	    glShadeModel(GL_SMOOTH);						        
	    glBindTexture(GL_TEXTURE_2D, mTexture);				// Select Our Texture
        glColor4f(1.0f,1.0f,1.0f,1.0f);
	    gluQuadricNormals(quadratic, GLU_SMOOTH);
        gluQuadricTexture(quadratic, GL_TRUE);
	    glBindTexture(GL_TEXTURE_2D, mTexture);				// Select Our Texture
    }  
    glRotatef(270.0f,1.0f,0.0f, 0.0f);
    gluSphere(quadratic,radius,nLongs,nLats);
    glPopMatrix();

    glRotatef(270.0f,1.0f,0.0f, 0.0f);

    // draw bold cardinal rings
    if (!bShowSphereGrid && !bWireframe)    // pano only
    {

    }
    else
    {
        float fZOffset = 0.008f;
        glColor4fv(matGreyLightLight);
        glLineWidth(2.0f);
        _drawCircle(0.0f,0.0f,radius-fZOffset, nLats, false);
        glPushMatrix();
        glRotatef(90.0f,1.0f,0.0f,0.0f);
        _drawCircle(0.0f,0.0f,radius-fZOffset, nLats, false);
        glRotatef(90.0f,0.0f,1.0f,0.0f);
        _drawCircle(0.0f,0.0f,radius-fZOffset, nLats, false);
        glPopMatrix();
        glLineWidth(1.0f);
    }

    // draw wireframe overlay over texture sphere
    if (bShowSphereGrid)
    {
        if(!bWireframe) 
        {
            glDisable(GL_TEXTURE_2D);						        
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	        glColor4fv(matWhiteClear);
            gluSphere(quadratic,radius-0.01f,nLongs,nLats);
        }
    }
    // draw lat/long labels
    if (!bShowSphereGrid && !bWireframe)    // pano only
    {

    }
    else
    {
	    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	    glEnable(GL_TEXTURE_2D);						        
        glShadeModel(GL_SMOOTH);						        
	    glRotatef(-270, 1.0f, 0.0f, 0.0f);  // negate orginal sphere rotation
        float fLat, fLong;
        float fPos[3];
        float fLabelOffsetX = 0.4f;
        // draw lat labels
	    for (fLat=-90; fLat <= 90 /*- m_deltaLat*/; fLat += (int)m_deltaLat)
	    {
            fPos[0] = 0.0f;
            fPos[1] = radius * sinf(fLat*DTOR);
            fPos[2] = radius * cosf(fLat*DTOR);
            glPushMatrix();
            glTranslatef(fPos[0]+fLabelOffsetX, fPos[1],-fPos[2]);
            glRotatef(fLat,1.0f,0.0f,0.0f);        // rotate to face (0,0,0)
		    glColor4fv(matWhiteClear);
            char szValue[512];
            sprintf_s(szValue, "%3.0f", -fLat);
            if(fLat == 0.0f)
                sprintf_s(szValue, "%3.0f", fLat);
			pRenderer->glPrint(0.0f, 0.0f, szValue);
            glPopMatrix();
        }

        // draw long labels
	    for (fLong=-180; fLong < 180 /*- m_deltaLong*/; fLong += (int)m_deltaLong)
	    {
            if(fLong == 0.0f)
                continue;
            fPos[0] = radius * sinf(fLong*DTOR);
            fPos[1] = 0.0f;
            fPos[2] = radius * cosf(fLong*DTOR);
            glPushMatrix();
            glTranslatef(fPos[0]+fLabelOffsetX, fPos[1],-fPos[2]);
            glRotatef(-fLong,0.0f,1.0f,0.0f);     // rotate to face (0,0,0)
		    glColor4fv(matWhiteClear);
            char szValue[512];
            sprintf_s(szValue, "%3.0f", -fLong);
			pRenderer->glPrint(0.0f, 0.0f, szValue);
            glPopMatrix();
        }
    }

 	glDisable(GL_TEXTURE_2D);						        
   
    // draw reticle
    float fCircleRadius = 4.0f;
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_BLEND);
    glPushMatrix();
    float matColor[4];
    matcpy(matColor, matYellowClear);
    matColor[3] = .25f;
    glColor4fv(matColor);
    glLineWidth(1.0f);
    float fOffset = 0.8f;

    // trackir ori
    glTranslatef(this->Position[0], this->Position[1], this->Position[2]);
    glRotatef(this->Rotation[2], 0.0f, 0.0f, 1.0f);
    glRotatef(this->Rotation[1], 0.0f, 1.0f, 0.0f);
    glRotatef(-this->Rotation[0], 1.0f, 0.0f, 0.0f);

	glTranslatef(0.0f,0.0f,-radius+fOffset);
    // view "highlight" circle
    _drawCircle(0.0f,0.0f,3.0f*fCircleRadius, nLats, true);
    // reticle
    glTranslatef(0.0f,0.0f,fOffset);    // offset from highlight
    glColor4fv(matWhiteClear);
    _drawCircle(0.0f,0.0f,fCircleRadius, nLats, false);
    _drawCircle(0.0f,0.0f,fCircleRadius*.2f, nLats, false);
    float fLength = fCircleRadius * 1.2f;
    glBegin(GL_LINES);
        glVertex3f(-fLength,0.0f,0.0f);
        glVertex3f(fLength,0.0f,0.0f);
        glVertex3f(0.0f,-fLength,0.0f);
        glVertex3f(0.0f,fLength,0.0f);
    glEnd();

    glPopMatrix();

    glEnable(GL_CULL_FACE);
    glPopMatrix();
	glPopAttrib();
}


///////////////////////////////////////////////////////////////////////////////
// glTriad
///////////////////////////////////////////////////////////////////////////////

glTriad::glTriad(void)
{
	quadratic = gluNewQuadric();			    
	gluQuadricNormals(quadratic, GLU_SMOOTH);	
	gluQuadricTexture(quadratic, GL_TRUE);		
}

glTriad::~glTriad(void)
{
   	gluDeleteQuadric(quadratic);
}

void glTriad::Render(void* pClientData)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushMatrix();

    glEnable(GL_COLOR_MATERIAL);

    float fHeight = 5.0f;
    float fBaseRadius = fHeight * .05f;
    float fTopRadius = fBaseRadius;

    float fConeBaseRadius = 2.0f * fBaseRadius;
    float fConeTopRadius = 0.0f;
    float fConeHeight = .3f * fHeight;

    float fFontSize = 1.2f;

    // trackir ori

    // relative
    glTranslatef(this->Position[0], this->Position[1], this->Position[2]);
    glRotatef(this->Rotation[2], 0.0f, 0.0f, 1.0f);
    glRotatef(this->Rotation[1], 0.0f, 1.0f, 0.0f);
    glRotatef(-this->Rotation[0], 1.0f, 0.0f, 0.0f);

    // X AXIS
    glPushMatrix();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  	glRotatef(90,0.0f,1.0f,0.0f);			        // default orientation is down +z axis - rotate to +x
    // cylinder
    glColor4fv(matRedClear);
    gluCylinder(quadratic,fBaseRadius,fTopRadius,fHeight,32,1);
    glColor4fv(matGreyLightClear);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // cap
    glPushMatrix();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor4fv(matRedClear);
    glTranslatef(0.0f,0.0f,fHeight);
    gluCylinder(quadratic,fConeBaseRadius,fConeTopRadius,fConeHeight,8,1);
    glPopMatrix();
    // label
    glPushMatrix();
  	glRotatef(-90,0.0f,1.0f,0.0f);
	glColor4fv(matWhiteSolid);
	glPopMatrix(); // label
    glPopMatrix(); // axis


    // Y AXIS
    glPushMatrix();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  	glRotatef(270.0f,1.0f,0.0f,0.0f);			        // default orientation is down +z axis - rotate to +y
    // cylinder
    glColor4fv(matBlueClear);
    gluCylinder(quadratic,fBaseRadius,fTopRadius,fHeight,32,1);
    glColor4fv(matGreyLightClear);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // cap
    glPushMatrix();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor4fv(matBlueClear);
    glTranslatef(0.0f,0.0f,fHeight);
    gluCylinder(quadratic,fConeBaseRadius,fConeTopRadius,fConeHeight,8,1);
    glPopMatrix();
    // label
    glPushMatrix();
  	glRotatef(90,1.0f,0.0f,0.0f);
  	glRotatef(-90,0.0f,1.0f,0.0f);
	glColor4fv(matWhiteSolid);
    glPopMatrix(); // label
    glPopMatrix(); // axis
    // Z AXIS
    glPushMatrix();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    // cylinder
    glColor4fv(matGreenClear);
    gluCylinder(quadratic,fBaseRadius,fTopRadius,fHeight,32,1);
    glColor4fv(matGreyLightClear);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // cap
    glPushMatrix();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor4fv(matGreenClear);
    glTranslatef(0.0f,0.0f,fHeight);
    gluCylinder(quadratic,fConeBaseRadius,fConeTopRadius,fConeHeight,8,1);
    glPopMatrix();
    // label
    glPushMatrix();
  	glRotatef(-90,0.0f,1.0f,0.0f);
	glColor4fv(matWhiteSolid);
    glPopMatrix(); // label
    glPopMatrix(); // axis

    // base sphere
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor4fv(matYellow);
    gluSphere(quadratic, fConeBaseRadius, 8,8);

	glDisable(GL_COLOR_MATERIAL);
    glPopMatrix();
	glPopAttrib();
}


///////////////////////////////////////////////////////////////////////////////
// glRenderer
///////////////////////////////////////////////////////////////////////////////

glRenderer::glRenderer(void)
{
    m_WindowWidth = 0;
    m_WindowHeight = 0;
    m_bPano = false;
    m_bTrackingView = true;
	memset(&tirData, 0, sizeof(TrackIRData));
    m_pView = new glView();
}

glRenderer::~glRenderer(void)
{
	if(m_pView)
		delete m_pView;
    _KillFont();
}

bool glRenderer::Initialize(HWND hWnd)
{
    BOOL bSuccess;
    int pixelFormat;
	m_hDC = ::GetDC(hWnd);
    if(!m_hDC)
        return false;

    RECT rect;
    ::GetWindowRect(hWnd, &rect);
    m_WindowWidth = rect.right - rect.left;
    m_WindowHeight = rect.bottom - rect.top;

    // Create OGL context
    static PIXELFORMATDESCRIPTOR pfd = {
        sizeof(PIXELFORMATDESCRIPTOR),  // size of this pfd
        1,                              // version number
        PFD_DRAW_TO_WINDOW |            // support window
        PFD_SUPPORT_OPENGL |	        // support OpenGL
        PFD_DOUBLEBUFFER ,			    // double buffered
        PFD_TYPE_RGBA,                  // RGBA type
        24,                             // 24-bit color depth
        0, 0, 0, 0, 0, 0,               // color bits ignored
        0,                              // no alpha buffer
        0,                              // shift bit ignored
        0,                              // no accumulation buffer
        0, 0, 0, 0,                     // accum bits ignored
        32,                             // 32-bit z-buffer      
        0,                              // no stencil buffer
        0,                              // no auxiliary buffer
        PFD_MAIN_PLANE,                 // main layer
        0,                              // reserved
        0, 0, 0                         // layer masks ignored
    };

    pixelFormat = ChoosePixelFormat(m_hDC, &pfd);
    if (!pixelFormat)
    {
        return false;
    }

    bSuccess = SetPixelFormat(m_hDC, pixelFormat, &pfd);
    if(!bSuccess)
    {
        return false;
    }

    m_hglrc = wglCreateContext( m_hDC );
    if (!m_hglrc)
    {
        return false;
    }

    if (!wglMakeCurrent( m_hDC, m_hglrc ))
    {
        wglDeleteContext( m_hglrc );
        m_hglrc = 0;
        return false;
    }

    // Create OGL LIGHTS
    // 1 Overhead, front, directional light
    GLfloat f4Ambience[] = { 0.75f, 0.75f, 0.75f, 1.0f};
    GLfloat f4Diffuse[]  = { .75f, .75f, .75f, 1.0f};
    GLfloat f4Diffuse2[] = { .1f, .1f, .1f, 0.5f };
    GLfloat f4Specular[] = { 1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat f4LightFront[4];
    f4LightFront[0] = 30.0f;
    f4LightFront[1] = 60.0f;
    f4LightFront[2] = 100.0f;
    f4LightFront[3] = 0.0f; // 0 indicates directional
    glLightfv(GL_LIGHT0, GL_AMBIENT, f4Diffuse2);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, f4Diffuse2);
    glLightfv(GL_LIGHT0, GL_SPECULAR, f4Specular);
    glLightfv(GL_LIGHT0, GL_POSITION, f4LightFront);
    // 1 ambient
    float f4Ambient[] = {0.4f,0.4f,0.4f,1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, f4Ambient);
	glEnable(GL_LIGHT0);

    // Initialize OGL rendering settings
    glClearColor(0.0f,0.0f,0.0f,0.0f);
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
    glEnable(GL_COLOR_MATERIAL);                        // Enable color material mode ( for 1:1 color matching with HUD/lines)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Enable nice perspective calcs
    glEnable(GL_NORMALIZE);                             // Enable autogenerate normals
    glEnable(GL_LINE_SMOOTH);                           // Enable Line antialiasing
    glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
    glCullFace(GL_BACK);                                // Enable backface culling
    glEnable(GL_CULL_FACE);
    
    // build a wgl font
    _BuildFont(m_hDC);

    m_pTriad = new glTriad();

    return true;
}

void glRenderer::SetPerspective(double dFOV, double dAspect, double dNear, double dFar)
{
    gluPerspective(dFOV, dAspect, dNear, dFar);
}

void glRenderer::SetOrtho(double dLeft, double dRight, double dBottom, double dTop, double dNear, double dFar)
{
    glOrtho(dLeft, dRight, dBottom, dTop, dNear, dFar);
}

void glRenderer::Render(void* pClientData)
{
    double dNear = 1.0f;
    double dFar = 15000.0f;
    double fov = 50.0f;

	// set ogl context
    BOOL bSuccess = wglMakeCurrent(m_hDC, m_hglrc);

    // clear background
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // prepare viewport
    int iLeft = 0;
	int iBottom = 0;
    int iWidth = m_WindowWidth;
    int iHeight = m_WindowHeight;
    glViewport(iLeft, iBottom, iWidth, iHeight);
    
    // prepare projection 
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    SetPerspective(fov, (double)m_WindowWidth/(double)m_WindowHeight, dNear, dFar);

    // prepare viewpoint
    m_pView->Projection = Perspective;
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	if (m_pView->Projection == Perspective)
    {
        if(m_bTrackingView)
        {
            // first person view - view assumes same pos/ori as tracked vector
            m_pView->EyeX = -1.0f * (-tirData.x);   // Trackir x is reversed
            m_pView->EyeY = - tirData.y;
            m_pView->EyeZ = - tirData.z;
            m_pView->EyeAzimuth = tirData.yaw;
            m_pView->EyeElevation = tirData.pitch;
            m_pView->EyeRoll = tirData.roll;

            // standard ogl approach - works, but will gimbal lock
            glRotated(m_pView->EyeElevation, 1.0, 0.0, 0.0);           // pitch
            glRotated(-m_pView->EyeAzimuth, 0.0, 1.0, 0.0);            // yaw   
            glRotated(-m_pView->EyeRoll, 0.0, 0.0, 1.0);               // roll
            glTranslated(m_pView->EyeX, m_pView->EyeY, m_pView->EyeZ);
        }
        else
        {
            // third person view - view orbits a fixed distance away from and looking at the origin
            glTranslated(0.0, 0.0, -m_pView->EyeDistance);
            glRotated(m_pView->EyeElevation, 1.0, 0.0, 0.0);           // pitch
            glRotated(-m_pView->EyeAzimuth, 0.0, 1.0, 0.0);            // yaw   
            glRotated(-m_pView->EyeRoll, 0.0, 0.0, 1.0);               // roll
        }
    }
    else 
    {
        glTranslated(0.0, -10.0, -50.0f);
    }


    // render origin triad
    // note - glprint in this render() messses up subsequent gl context's viewports
    // so restore after rendering
    m_pTriad->Render(pClientData);      
    glViewport(iLeft, iBottom, iWidth, iHeight);
    
	// render visual objects
    std::list<glObject*>::iterator iter;
    // render perspective objects list
    for(iter = m_RenderList3D.begin(); iter != m_RenderList3D.end(); iter++)
    {
        glObject* pObject = *iter;
        if(pObject->m_bVisible)
        {
            if(pObject->Projection == Orthographic)   // skip ortho objects
                continue;

            // render object
            glPushMatrix();
            pObject->Render(this);               
            glPopMatrix();
        }
    }
	// render ortho objects list
    for(iter = m_RenderList3D.begin(); iter != m_RenderList3D.end(); iter++)
    {
        glObject* pObject = *iter;
        if(pObject->m_bVisible)
        {
            if(pObject->Projection == Perspective)    // skip perspective objects
                continue;

            glPushMatrix();

            // prepare projection 
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            // window/pixel space
            double left = 0.0f;
            double right = m_WindowWidth;
            // gl style (0,0 = bottom left)
            double bottom = 0.0;
            double top = m_WindowHeight;           
            dNear = 1.0f;
            dFar = -1.0f;
            SetOrtho(left, right, bottom, top, dNear, dFar);

            // prepare viewpoint
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            // render object
            pObject->Render(this);

            glPopMatrix();
        }
    }

	// draw text status strings
    glPushMatrix();
    // prepare projection 
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // window/pixel space
    double left = 0.0f;
    double right = m_WindowWidth;	
    // ms windows style (0,0 = top left)
    double bottom = m_WindowHeight;
    double top = 0.0f;
    dNear = -1.0f;
    dFar = 1.0f;
    SetOrtho(left, right, bottom, top, dNear, dFar);
    // prepare viewpoint
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	glColor4fv(matYellowClear);
	int x1 = (int)left + 10;
	int x2 = (int)left + 55;
	int y1 = (int)top + 25;
	int sp = 22;
	
	glPrint(x1, y1,      "TrackIR Version : %s", szVersion);

	glPrint(x1, y1+2*sp, "Frame Signature : %d", NPFrameSignature);
	glPrint(x1, y1+3*sp, "Stale Frames : %d", NPStaleFrames);
#if LATENCY_TESTING
	glPrint(x1, y1+4*sp, "Latency : %3.2f (ms)", NPLatencyAverage);
#endif
	glPrint(x1, y1+5*sp, "Sampling Period : %3.2f (ms)", dSampleDeltaAverage);
	glPrint(x1, y1+6*sp, "Status  : %s", szStatus);

	glPrint(x1, y1+8*sp,  "X");
	glPrint(x2, y1+8*sp,  ":  %3.2f (cm)", tirData.x);
	glPrint(x1, y1+9*sp,  "Y");
	glPrint(x2, y1+9*sp,  ":  %3.2f (cm)", tirData.y);
	glPrint(x1, y1+10*sp,  "Z");
	glPrint(x2, y1+10*sp,  ":  %3.2f (cm)", tirData.z);

	glPrint(x1, y1+11*sp, "Yaw");
	glPrint(x2, y1+11*sp, ":  %3.2f (deg)", tirData.yaw);
	glPrint(x1, y1+12*sp, "Pitch");
	glPrint(x2, y1+12*sp, ":  %3.2f (deg)", tirData.pitch);
	glPrint(x1, y1+13*sp, "Roll");
	glPrint(x2, y1+13*sp, ":  %3.2f (deg)", tirData.roll);

	glPopMatrix();

    // swap buffers
	glFlush();   
    SwapBuffers(m_hDC);
    wglMakeCurrent(NULL, NULL);

}

void glRenderer::ResizeWindow(int width, int height)
{
    m_WindowWidth = width;
    m_WindowHeight = height;
}

bool glRenderer::MakeCurrent(bool bMakeCurrent)
{
    BOOL bSuccess;
    if(bMakeCurrent)
        bSuccess = wglMakeCurrent(m_hDC, m_hglrc);
    else
        bSuccess = wglMakeCurrent(NULL,NULL);
    return (bSuccess!=0);
}

bool _PointInRect(int x, int y, int left, int bottom, int right, int top)
{
    if(x < left) return false;
    if(x > right) return false;
    if(y < bottom) return false;
    if(y > top) return false;
    return true;
}

int glRenderer::OnMouseWheel(unsigned int nFlags, short zDelta, int x, int y)
{
    int iHandled = 0;
    int pointX = x;
    int pointY = m_WindowHeight - y;

    // pass mouse event to any object that
    // - wants the mouse
    // - is under the cursor
    std::list<glObject*>::iterator iter;
    for(iter = m_RenderList3D.begin(); iter != m_RenderList3D.end(); iter++)
    {
        glObject* pObject = *iter;
        if(pObject->bWantMouse)
        {
            int left, bottom, right, top;
            pObject->GetExtents(&left, &bottom, &right, &top);  // from bottom left, in WCUs
            bool bHit = _PointInRect(pointX, pointY, left, bottom, right, top);
            if(bHit)
            {
                iHandled = pObject->OnMouseWheel(nFlags, zDelta, pointX, pointY);
                if(iHandled > 0)
                    break;
            }
        }
    }

    return iHandled;
}

int glRenderer::OnMouseDown(unsigned int nFlags, int x, int y)
{
    int iHandled = 0;
    int pointX = x;
    int pointY = m_WindowHeight - y;

    // pass mouse event to any object that
    // - wants the mouse
    // - is under the cursor
    std::list<glObject*>::iterator iter;
    for(iter = m_RenderList3D.begin(); iter != m_RenderList3D.end(); iter++)
    {
        glObject* pObject = *iter;
        if(pObject->bWantMouse)
        {
            int left, bottom, right, top;
            pObject->GetExtents(&left, &bottom, &right, &top);  // from bottom left, in WCUs
            bool bHit = _PointInRect(pointX, pointY, left, bottom, right, top);
            if(bHit)
            {
                iHandled = pObject->OnMouseDown(nFlags, pointX, pointY);
                if(iHandled > 0)
				{
					// do something here....
					//m_pSelectedObject = pObject;	// some client may want to know what was clicked. store it here for them
                    break;
				}
            }
        }
    }

    return iHandled;
}

int glRenderer::OnMouseMove(unsigned int nFlags, int x, int y)
{
    int iHandled = 0;
    int pointX = x;
    int pointY = m_WindowHeight - y;
	bool bHit = false;
	bool bNeedRedraw = false;
	int hitCount = 0;

    // pass mouse event to any object that
    // - wants the mouse
    // - is under the cursor
    std::list<glObject*>::iterator iter;
    for(iter = m_RenderList3D.begin(); iter != m_RenderList3D.end(); iter++)
    {
        glObject* pObject = *iter;
        if(pObject->bWantMouse)
        {
            int left, bottom, right, top;
            pObject->GetExtents(&left, &bottom, &right, &top);  // from bottom left, in WCUs
            bHit = _PointInRect(pointX, pointY, left, bottom, right, top);
            if(bHit)
            {
				// todo: handle your mouse move here...
            }
        }
    }

    return iHandled;
}

int glRenderer::OnMouseUp(unsigned int nFlags, int x, int y)
{
    int iHandled = 0;
    int pointX = x;
    int pointY = m_WindowHeight - y;

    // pass mouse event to any object that
    // - wants the mouse
    // - is under the cursor
    std::list<glObject*>::iterator iter;
    for(iter = m_RenderList3D.begin(); iter != m_RenderList3D.end(); iter++)
    {
        glObject* pObject = *iter;
        if(pObject->bWantMouse)
        {
            int left, bottom, right, top;
            pObject->GetExtents(&left, &bottom, &right, &top);  // from bottom left, in WCUs
            bool bHit = _PointInRect(pointX, pointY, left, bottom, right, top);
            if(bHit)
            {
                iHandled = pObject->OnMouseUp(nFlags, pointX, pointY);
                if(iHandled > 0)
                    break;
            }
        }
    }

    return iHandled;
}

int glRenderer::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
    int iHandled = 0;

    std::list<glObject*>::iterator iter;
    for(iter = m_RenderList3D.begin(); iter != m_RenderList3D.end(); iter++)
    {
        glObject* pObject = *iter;
        if(pObject->bWantKeyboard)
        {
			iHandled = pObject->OnKeyDown(nChar, nRepCnt, nFlags);
            if(iHandled > 0)
				break;
        }
    }

    return iHandled;

}


// draw wgl text specified in BuildFont()
// note : 
//  - check/set raster position before calling glPrint
//  - call build font only after a *valid* DC has been set
void glRenderer::glPrint(double x, double y, const char *fmt, ...)
{
	char		text[256];								
	va_list		ap;										
	if (fmt == NULL)									
		return;											

	// parse formatted string/args into text string
	va_start(ap, fmt);									
	    vsprintf_s(text, fmt, ap);					
	va_end(ap);											

    // wgl text
    glPushMatrix();
    glTranslated(x,y,0.0f);
	// rotate and scale cannot be aplied to wgl bitmap fonts, only wgl outline fonts ( polys )
	//glRotated(angle,0.0,0.0,1.0);
	//glScalef(0.02,0.02,0.02);
	glRasterPos2d(0.0,0.0);
	// draw the text
	glListBase(base - 32);								
	glCallLists((GLsizei)strlen(text), GL_UNSIGNED_BYTE, text);	
	glPopMatrix();
}

// Build the bitmap font display list using wgl
void glRenderer::_BuildFont(HDC hDC)
{
    HFONT	font;										
	HFONT	oldfont;								
	base = glGenLists(96);								// Storage For 96 Characters

    m_font.lfHeight          = -16;     
    m_font.lfWidth           = 0; 
    m_font.lfEscapement      = 0; 
    m_font.lfOrientation     = 0; 
    m_font.lfWeight          = FW_MEDIUM; //FW_NORMAL; 
    m_font.lfItalic          = FALSE; 
    m_font.lfUnderline       = FALSE; 
    m_font.lfStrikeOut       = FALSE; 
    m_font.lfCharSet         = ANSI_CHARSET; 
    m_font.lfOutPrecision    = OUT_TT_PRECIS; 
    m_font.lfClipPrecision   = CLIP_DEFAULT_PRECIS; 
    m_font.lfQuality         = ANTIALIASED_QUALITY; 
    m_font.lfPitchAndFamily  = FF_DONTCARE | DEFAULT_PITCH; 
    lstrcpy(m_font.lfFaceName, TEXT("Arial"));
    font = CreateFontIndirect( &m_font );
	oldfont = (HFONT)SelectObject(hDC, font);           

	BOOL bSuccess = wglUseFontBitmaps(hDC, 32, 96, base);

	// we're done with the font object so release it
    SelectObject(hDC, oldfont);
	DeleteObject(font);
}

// Delete the font list
void glRenderer::_KillFont(void)
{
    glDeleteLists(base, 96);    // wgl font - Delete All 96 Characters
}

