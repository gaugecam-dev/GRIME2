/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   Copyright 2021 Kenneth W. Chapman

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#ifndef LABELROI_H
#define LABELROI_H

#include "log.h"
#include "gc_types.h"
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/exception.hpp>
#include <boost/exception/diagnostic_information.hpp>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Annotator we are using: VGG
// Download: http://www.robots.ox.ac.uk/~vgg/software/via/
// Tutorial: https://www.youtube.com/watch?v=-3WVSxNLk_k
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = boost::filesystem;

namespace gc
{

class LabelROIItem
{
public:
    LabelROIItem() {}
    LabelROIItem( const std::string roiName,
                  const std::string roiType,
                  const std::vector< cv::Point > contourPts,
                  const cv::RotatedRect ellipse,
                  const cv::Scalar rgbColor ) :
        name( roiName ), roi_type( roiType ), contour( contourPts ),
        rotRect( ellipse ), color( rgbColor ) {}

    void clear()
    {
        name.clear();
        roi_type.clear();
        contour.clear();
        rotRect = cv::RotatedRect();
        color = cv::Scalar( 0, 255, 255 );
    }

    std::string name;
    std::string roi_type;
    std::vector< cv::Point > contour;
    cv::RotatedRect rotRect;
    cv::Scalar color;
};

class LabelROI
{
public:
    LabelROI() {}

    static GC_STATUS Load( const std::string jsonFilepath, std::vector< LabelROIItem > &labeledRois )
    {
        GC_STATUS retVal = GC_OK;
        try
        {
            if ( ".json" != jsonFilepath.substr( jsonFilepath.size() - 5 ) )
            {
                FILE_LOG( logERROR ) << "[LabelROI::Load] Json filepath mus have a \".json\" extension: " << jsonFilepath;
                retVal = GC_ERR;
            }
            else if ( !fs::exists( jsonFilepath ) )
            {
                FILE_LOG( logERROR ) << "[LabelROI::Load] Json file does not exist: " << jsonFilepath;
                retVal = GC_ERR;
            }
            else if ( !fs::is_regular_file( jsonFilepath ) )
            {
                FILE_LOG( logERROR ) << "[LabelROI::Load] Filepath is not a file: " << jsonFilepath;
                retVal = GC_ERR;
            }
            else
            {
                labeledRois.clear();

                property_tree::ptree pt, ptRoi, ptAttr;
                property_tree::json_parser::read_json( jsonFilepath, pt );
                property_tree::ptree ptMeta = pt.get_child( "_via_img_metadata" );

                Rect rect;
                LabelROIItem item;

                int b, g, r;
                std::srand( static_cast< unsigned int >( std::time( nullptr ) ) );

                for ( const auto &region : ptMeta.get_child( "regions" ) )
                {
                    item.clear();
                    b = cvRound( 255.0 * static_cast< double >( std::rand() ) / static_cast< double >( RAND_MAX ) );
                    g = cvRound( 255.0 * static_cast< double >( std::rand() ) / static_cast< double >( RAND_MAX ) );
                    r = cvRound( 255.0 * static_cast< double >( std::rand() ) / static_cast< double >( RAND_MAX ) );
                    item.color = Scalar( b, g, r );

                    ptAttr = region.second.get_child( "region_attributes" );
                    item.name = ptAttr.get< string >( "ROI" );
                    ptAttr = region.second.get_child( "shape_attributes" );
                    item.roi_type = ptAttr.get< string >( "name" );
                    if ( "rect" == item.roi_type )
                    {
                        rect.x = ptAttr.get< int >( "x" );
                        rect.y = ptAttr.get< int >( "y" );
                        rect.width = ptAttr.get< int >( "width" );
                        rect.height = ptAttr.get< int >( "height" );
                        item.contour.push_back( Point( rect.x, rect.y ) );
                        item.contour.push_back( Point( rect.x + rect.width - 1, rect.y ) );
                        item.contour.push_back( Point( rect.x + rect.width - 1, rect.y + rect.height - 1 ) );
                        item.contour.push_back( Point( rect.x, rect.y + rect.height - 1 ) );
                        labeledRois.push_back( item );
                    }
                    else if ( "ellipse" == item.roi_type )
                    {
                        item.rotRect.center.x = ptAttr.get< float >( "cx" );
                        item.rotRect.center.y = ptAttr.get< float >( "cy" );
                        item.rotRect.size.width = ptAttr.get< float >( "rx" );
                        item.rotRect.size.height = ptAttr.get< float >( "ry" );
                        item.rotRect.angle = ptAttr.get< float >( "theta" );
                        labeledRois.push_back( item );
                    }
                    else if ( "polygon" == item.roi_type )
                    {
                        for ( const auto &coord : ptAttr.get_child( "all_points_x" ) )
                        {
                            item.contour.push_back( Point( coord.second.get_value< int >(), -999 ) );
                        }
                        int idx = 0;
                        for ( const auto &coord : ptAttr.get_child( "all_points_y" ) )
                        {
                            item.contour[ idx++ ].y = coord.second.get_value< int >();
                        }
                        labeledRois.push_back( item );
                    }
                    else
                    {
                        FILE_LOG( logWARNING ) << "Region type \"" << item.roi_type << "\" not yet accommodated";
                    }
                }
            }
        }
        catch( const boost::exception &e )
        {
            FILE_LOG( logERROR ) << "[LabelROI::Load] " << diagnostic_information( e );
            retVal = GC_EXCEPT;
        }
        return retVal;
    }
};

} // namespace gc

#endif // LABELROI_H

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Example json
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
{
  "_via_settings": {
    "ui": {
      "annotation_editor_height": 25,
      "annotation_editor_fontsize": 0.8,
      "leftsidebar_width": 18,
      "image_grid": {
        "img_height": 80,
        "rshape_fill": "none",
        "rshape_fill_opacity": 0.3,
        "rshape_stroke": "yellow",
        "rshape_stroke_width": 2,
        "show_region_shape": true,
        "show_image_policy": "all"
      },
      "image": {
        "region_label": "__via_region_id__",
        "region_color": "__via_default_region_color__",
        "region_label_font": "10px Sans",
        "on_image_annotation_editor_placement": "NEAR_REGION"
      }
    },
    "core": {
      "buffer_size": 18,
      "filepath": {},
      "default_filepath": ""
    },
    "project": {
      "name": "VGG_NCMarch_23Nov2020_14h59m"
    }
  },
  "_via_img_metadata": {
    "NRmarshDN_reference_image.jpg88394": {
      "filename": "NRmarshDN_reference_image.jpg",
      "size": 88394,
      "regions": [
        {
          "shape_attributes": {
            "name": "rect",
            "x": 4,
            "y": 4,
            "width": 268,
            "height": 593
          },
          "region_attributes": {
            "ROI": "whole_image"
          }
        },
        {
          "shape_attributes": {
            "name": "rect",
            "x": 6,
            "y": 8,
            "width": 260,
            "height": 329
          },
          "region_attributes": {
            "ROI": "bank_area"
          }
        },
        {
          "shape_attributes": {
            "name": "rect",
            "x": 8,
            "y": 341,
            "width": 260,
            "height": 253
          },
          "region_attributes": {
            "ROI": "water_area"
          }
        }
      ],
      "file_attributes": {}
    }
  },
  "_via_attributes": {
    "region": {
      "ROI": {
        "type": "radio",
        "description": "Region of interest",
        "options": {
          "whole_image": "",
          "water_area": "",
          "bank_area": ""
        },
        "default_options": {}
      }
    },
    "file": {}
  },
  "_via_data_format_version": "2.0.10",
  "_via_image_id_list": [
    "NRmarshDN_reference_image.jpg88394"
  ]
}
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
