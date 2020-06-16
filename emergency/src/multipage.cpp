#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include "hpdf.h"
#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <ros/package.h>

jmp_buf env;
using namespace std;

#ifdef HPDF_DLL
void  __stdcall
#else
void
#endif
error_handler  (HPDF_STATUS   error_no,
                HPDF_STATUS   detail_no,
                void         *user_data)
{
    printf ("ERROR: error_no=%04X, detail_no=%u\n", (HPDF_UINT)error_no,
                (HPDF_UINT)detail_no);
    longjmp(env, 1);
}


void
print_page  (HPDF_Page   page,  int page_num)
{
    char buf[50];

    HPDF_Page_SetWidth (page, 800);
    HPDF_Page_SetHeight (page, 800);

    HPDF_Page_BeginText (page);
    HPDF_Page_MoveTextPos (page, 50, 740);
    snprintf(buf, 50, "Page:%d", page_num);
   
    HPDF_Page_ShowText (page, buf);
    HPDF_Page_EndText (page);
}
void print_page_text  (HPDF_Page   page,  string texts,int i)
{
    char buf[50];

    HPDF_Page_SetWidth (page, 800);
    HPDF_Page_SetHeight (page, 800);

    HPDF_Page_BeginText (page);
    HPDF_Page_MoveTextPos (page, 50, 740-i);
    
    snprintf(buf, 50, texts.c_str());
    HPDF_Page_ShowText (page, buf);
    HPDF_Page_EndText (page);
}

void
draw_image (HPDF_Doc     pdf,
            const char  *filename,
            float        x,
            float        y,
            const char  *text)
{
    const char* FILE_SEPARATOR = "/";
    
    char filename1[255];

    HPDF_Page page = HPDF_GetCurrentPage (pdf);
    HPDF_Image image;

    strcpy(filename1, "");
    strcat(filename1, FILE_SEPARATOR);
    strcat(filename1, filename);

    image = HPDF_LoadPngImageFromFile (pdf, filename1);

    /* Draw image to the canvas. */
    HPDF_Page_DrawImage (page, image, x, y, HPDF_Image_GetWidth (image),
                    HPDF_Image_GetHeight (image));

}

bool generatePDF(string path1,string path2,string path3,string path4,string outPutPath){
	HPDF_Doc  pdf;
    HPDF_Font font;
    HPDF_Page page[4];
    HPDF_Outline root;
    HPDF_Outline outline[4];
    HPDF_Destination dst;
    char fname[256];

    strcpy (fname,outPutPath.c_str());
    strcat (fname, ".pdf");

    pdf = HPDF_New (error_handler, NULL);
    if (!pdf) {
        printf ("error: cannot create PdfDoc object\n");
        return 1;
    }
    if (setjmp(env)) {
        HPDF_Free (pdf);
        return 1;
    }

    /* create default-font */
    font = HPDF_GetFont (pdf, "Helvetica", NULL);

    /* Set page mode to use outlines. */
    HPDF_SetPageMode(pdf, HPDF_PAGE_MODE_USE_OUTLINE);

    /* Add 3 pages to the document. */
    page[0] = HPDF_AddPage (pdf);
    HPDF_Page_SetFontAndSize (page[0], font, 30);
    //print_page(page[0], 1);
 
    draw_image (pdf,path1.c_str(), 50, HPDF_Page_GetHeight (page[0]) - 350,"");

    page[1] = HPDF_AddPage (pdf);
    HPDF_Page_SetFontAndSize (page[1], font, 30);
    //print_page(page[1], 2);
    
    draw_image (pdf,path2.c_str(), 50, HPDF_Page_GetHeight (page[1]) - 350,"");

    page[2] = HPDF_AddPage (pdf);
    HPDF_Page_SetFontAndSize (page[2], font, 30);
    //print_page(page[2], 3);
    draw_image (pdf,path3.c_str(), 50, HPDF_Page_GetHeight (page[2]) - 350,"");

    page[3] = HPDF_AddPage (pdf);
    HPDF_Page_SetFontAndSize (page[3], font, 30);
    //print_page(page[3], 4);
    ifstream file(path4.c_str());
    string str; 
    int i=50;
    while (getline(file, str))
    {
		
        print_page_text(page[3],str,i);
        i+=50;
    }   
    
    
    
    /* create outline root. */
    root = HPDF_CreateOutline (pdf, NULL, "OutlineRoot", NULL);
    HPDF_Outline_SetOpened (root, HPDF_TRUE);

    outline[0] = HPDF_CreateOutline (pdf, root, "page1", NULL);
    outline[1] = HPDF_CreateOutline (pdf, root, "page2", NULL);

    /* create outline with test which is ISO8859-2 encoding */
    outline[2] = HPDF_CreateOutline (pdf, root, "ISO8859-2 text ÓÔÕÖ×ØÙ",
                    HPDF_GetEncoder (pdf, "ISO8859-2"));

    /* create destination objects on each pages
     * and link it to outline items.
     */
    dst = HPDF_Page_CreateDestination (page[0]);
    HPDF_Destination_SetXYZ(dst, 0, HPDF_Page_GetHeight(page[0]), 1);
    HPDF_Outline_SetDestination(outline[0], dst);
  //  HPDF_Catalog_SetOpenAction(dst);

    dst = HPDF_Page_CreateDestination (page[1]);
    HPDF_Destination_SetXYZ(dst, 0, HPDF_Page_GetHeight(page[1]), 1);
    HPDF_Outline_SetDestination(outline[1], dst);

    dst = HPDF_Page_CreateDestination (page[2]);
    HPDF_Destination_SetXYZ(dst, 0, HPDF_Page_GetHeight(page[2]), 1);
    HPDF_Outline_SetDestination(outline[2], dst);
    
    dst = HPDF_Page_CreateDestination (page[3]);
    HPDF_Destination_SetXYZ(dst, 0, HPDF_Page_GetHeight(page[3]), 1);
    HPDF_Outline_SetDestination(outline[3], dst);
    /* save the document to a file */
    HPDF_SaveToFile (pdf, fname);

    /* clean up */
    HPDF_Free (pdf);
    return true;
}

int main(int argc, char **argv)
{

	string path1 = ros::package::getPath("emergency") +"/png/slam1.png";
	string path2 = ros::package::getPath("emergency") +"/png/slammark.png";
	string path3 = ros::package::getPath("emergency") +"/png/kinect.png";;
	string path4 = ros::package::getPath("emergency") +"/src/status.txt";
    string outPutPath=ros::package::getPath("emergency") +"/pdf/file";
    bool back=generatePDF(path1,path2,path3,path4,outPutPath);
    
    //cv::waitKey(50000000);
    sleep(2);
    //Send To Flash Memory
    if (back){
		string  mountPath = ros::package::getPath("emergency") +"/bash/mount1.sh";
          system(mountPath.c_str());
          
				mountPath = ros::package::getPath("emergency") +"/bash/mount2.sh";
		  system(mountPath.c_str());
		  
				mountPath = ros::package::getPath("emergency") +"/bash/mount3.sh";;
		  system(mountPath.c_str());      
	}
	else{
		cout<<"File NOT Generated!!"<<endl;
	}
    return 0;
}
