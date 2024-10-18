// Author of DualQuatFunctios: Edison Velasco 
// Email epvelasco1912@gmail.com, edison.velasco@ua.es

// Internal library
#include "STDesc.h"
#include "KDTree_STD.h"
#include <random> // only for plot arrows with differents color

#ifndef _STD_FUNCTIONS_H_
#define _STD_FUNCTIONS_H_

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

void convertToMarkers(const std::vector<STDesc>& stds, visualization_msgs::MarkerArray& marker_array, const Eigen::Vector3f& color, float alpha = 1.0, float scale = 0.03) {
    int id = 0;

    for (const auto& std : stds) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "std_descriptors";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = scale;
        marker.color.r = color(0);  
        marker.color.g = color(1);  
        marker.color.b = color(2);  
        marker.color.a = alpha;     

        geometry_msgs::Point p1, p2, p3;
        p1.x = std.vertex_A_(0);
        p1.y = std.vertex_A_(1);
        p1.z = std.vertex_A_(2);
        p2.x = std.vertex_B_(0);
        p2.y = std.vertex_B_(1);
        p2.z = std.vertex_B_(2);
        p3.x = std.vertex_C_(0);
        p3.y = std.vertex_C_(1);
        p3.z = std.vertex_C_(2);

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
        marker.points.push_back(p3);
        marker.points.push_back(p1);

        marker_array.markers.push_back(marker);
    }
}

void MAPconvertToMarkers(const Eigen::MatrixXf& data, visualization_msgs::MarkerArray& marker_array, const Eigen::Vector3f& color, float alpha = 1.0, float scale = 0.03) {
    int id = 0;

    for (int i = 0; i < data.rows(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "std_descriptors";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = scale;
        marker.color.r = color(0);  
        marker.color.g = color(1);  
        marker.color.b = color(2);  
        marker.color.a = alpha;     

        geometry_msgs::Point p1, p2, p3;
        p1.x = data(i,9);
        p1.y = data(i,10);
        p1.z = data(i,11);
        p2.x = data(i,12);
        p2.y = data(i,13);
        p2.z = data(i,14);
        p3.x = data(i,15);
        p3.y = data(i,16);
        p3.z = data(i,17);

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
        marker.points.push_back(p3);
        marker.points.push_back(p1);

        marker_array.markers.push_back(marker);
    }
}

void convertToPointCloud(const std::vector<STDesc>& stds, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    for (const auto& std : stds) {
        pcl::PointXYZ p1, p2, p3;
        p1.x = std.vertex_A_(0);
        p1.y = std.vertex_A_(1);
        p1.z = std.vertex_A_(2);
        p2.x = std.vertex_B_(0);
        p2.y = std.vertex_B_(1);
        p2.z = std.vertex_B_(2);
        p3.x = std.vertex_C_(0);
        p3.y = std.vertex_C_(1);
        p3.z = std.vertex_C_(2);

        cloud->points.push_back(p1);
        cloud->points.push_back(p2);
        cloud->points.push_back(p3);
    }
}

void MAPconvertToPointCloud(const Eigen::MatrixXf& data, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    for (int i = 0; i < data.rows(); ++i) {
        pcl::PointXYZ p1, p2, p3;

        // Asumiendo que los vertices estan en las posiciones 9-17
        p1.x = data(i, 9);
        p1.y = data(i, 10);
        p1.z = data(i, 11);
        
        p2.x = data(i, 12);
        p2.y = data(i, 13);
        p2.z = data(i, 14);

        p3.x = data(i, 15);
        p3.y = data(i, 16);
        p3.z = data(i, 17);

        cloud->points.push_back(p1);
        cloud->points.push_back(p2);
        cloud->points.push_back(p3);
    }
}

void printSTDesc(const STDesc& desc) {
    std::cout << "Side Lengths: " << desc.side_length_.transpose() << std::endl;
    std::cout << "Angles: " << desc.angle_.transpose() << std::endl;
    std::cout << "Center: " << desc.center_.transpose() << std::endl;
    std::cout << "Vertex A: " << desc.vertex_A_.transpose() << std::endl;
    std::cout << "Vertex B: " << desc.vertex_B_.transpose() << std::endl;
    std::cout << "Vertex C: " << desc.vertex_C_.transpose() << std::endl;
    //std::cout << "Norms : " << desc.norms.transpose() << std::endl;
    std::cout << "Frame ID: " << desc.frame_id_ << std::endl;
}

template <typename T>
void printVector(const std::vector<T>& vec) {
    std::cout << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << vec[i];
        if (i != vec.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}

void addDescriptorToMatrix(Eigen::MatrixXf& mat, const STDesc& desc, int row) {

    // la matriz tiene 24 elementos
    Eigen::Vector3f side_length = desc.side_length_.cast<float>();
  //  Eigen::Vector3f angle = desc.angle_.cast<float>();
    Eigen::Vector3f center = desc.center_.cast<float>();
    Eigen::Vector3f vertex_A = desc.vertex_A_.cast<float>();
    Eigen::Vector3f vertex_B = desc.vertex_B_.cast<float>();
    Eigen::Vector3f vertex_C = desc.vertex_C_.cast<float>();
    Eigen::Vector3f normal1 = desc.normal1_.cast<float>();
    Eigen::Vector3f normal2 = desc.normal2_.cast<float>();
    Eigen::Vector3f normal3 = desc.normal3_.cast<float>();
    //Eigen::Matrix3d axes = desc.calculateReferenceFrame();
   // Eigen::Matrix<float, 9, 1> axes_vec;
   // axes_vec << axes(0),axes(1),axes(2),axes(3),axes(4),axes(5),axes(6),axes(7),axes(8);
    mat.block<1, 3>(row, 0) = side_length.transpose();
    //mat.block<1, 3>(row, 3) = angle.transpose();
    mat.block<1, 3>(row, 3) = center.transpose();
    mat.block<1, 3>(row, 6) = vertex_A.transpose();
    mat.block<1, 3>(row, 9) = vertex_B.transpose();
    mat.block<1, 3>(row, 12) = vertex_C.transpose();
    mat.block<1, 3>(row, 15) = normal1.transpose();
    mat.block<1, 3>(row, 18) = normal2.transpose();
    mat.block<1, 3>(row, 21) = normal3.transpose();
    //mat.block<1, 9>(row, 27) = axes_vec.transpose();
}

void updateMatrixAndKDTree(Eigen::MatrixXf& mat, std::unique_ptr<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>>& index, const std::deque<STDesc>& std_local_map) {
    int num_desc = std_local_map.size();
    mat.resize(num_desc, 24);

    // Rellenar la matriz con los descriptores actuales
    for (size_t i = 0; i < std_local_map.size(); ++i) {
        addDescriptorToMatrix(mat, std_local_map[i], i);
    }

    // Recrear el KD-Tree con la matriz actualizada
    index = std::make_unique<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>>(24, std::cref(mat), 10 /* max leaf */);
    index->index_->buildIndex();
}

void publishLocalMap(const std::deque<STDesc>& std_local_map, visualization_msgs::MarkerArray& marker_array, const Eigen::Vector3f& color, float alpha = 1.0) {
    std::vector<STDesc> temp_vector;
    temp_vector.reserve(std_local_map.size());
    for (const auto& desc : std_local_map) {
        temp_vector.push_back(desc);
    }
   
    convertToMarkers(temp_vector, marker_array, color, alpha,0.06);
}

// Función para generar colores aleatorios
std::tuple<float, float, float> getRandomColor() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, 1.0);
    return std::make_tuple(dis(gen), dis(gen), dis(gen));
}

void generateArrow(const STDesc& desc1, const STDesc& desc2, visualization_msgs::MarkerArray& marker_array, int& id, const std_msgs::Header& header) {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.ns = "std_matches";
    arrow.id = id++;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.scale.x = 0.05;  // Grosor del cuerpo de la flecha
    arrow.scale.y = 0.2;  // Grosor de la cabeza de la flecha
    arrow.scale.z = 0.4;   // Longitud de la cabeza de la flecha
    
    // Generar color aleatorio
    auto [r, g, b] = getRandomColor();
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    // Punto de inicio (centro del descriptor 1)
    geometry_msgs::Point start;
    start.x = desc1.center_(0);
    start.y = desc1.center_(1);
    start.z = desc1.center_(2);

    // Punto final (centro del descriptor 2)
    geometry_msgs::Point end;
    end.x = desc2.center_(0);
    end.y = desc2.center_(1);
    end.z = desc2.center_(2);

    arrow.points.push_back(start);
    arrow.points.push_back(end);

    marker_array.markers.push_back(arrow);
}

// Función para calcular la distancia euclidiana entre dos vértices
float calcularDistancia(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2) {
    return (v1 - v2).norm();
}

void extractVerticesToMatrix(const std::deque<STDesc>& std_local_map, Eigen::MatrixXf& all_vertices) {
    int num_desc = std_local_map.size();
    all_vertices.resize(3 * num_desc, 3); // 3 filas por descriptor, cada una con 3 coordenadas

    for (int i = 0; i < num_desc; ++i) {
        all_vertices.row(3 * i) = std_local_map[i].vertex_A_.transpose().cast<float>();   // vertex_A
        all_vertices.row(3 * i + 1) = std_local_map[i].vertex_B_.transpose().cast<float>(); // vertex_B
        all_vertices.row(3 * i + 2) = std_local_map[i].vertex_C_.transpose().cast<float>(); // vertex_C
    }
}

void build_std_filter( const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points, std::vector<STDesc> &stds_vec, const ConfigSetting &config_setting_) {

    double scale = 1.0 / config_setting_.std_side_resolution_;   
    unsigned int current_frame_id_ = 0; // Se asume que este valor se incrementa en cada llamada a la función

    for (size_t i = 0; i < corner_points->size(); i += 3) {
        if (i + 2 >= corner_points->size()) break; // Asegurarse de que haya al menos tres puntos para formar un triángulo

        pcl::PointXYZINormal p1 = corner_points->points[i];
        pcl::PointXYZINormal p2 = corner_points->points[i + 1];
        pcl::PointXYZINormal p3 = corner_points->points[i + 2];

        Eigen::Vector3d A, B, C;
        A << p1.x, p1.y, p1.z;
        B << p2.x, p2.y, p2.z;
        C << p3.x, p3.y, p3.z;

        double a = (A - B).norm();
        double b = (A - C).norm();
        double c = (B - C).norm();

        Eigen::Vector3d normal_1(p1.normal_x, p1.normal_y, p1.normal_z);
        Eigen::Vector3d normal_2(p2.normal_x, p2.normal_y, p2.normal_z);
        Eigen::Vector3d normal_3(p3.normal_x, p3.normal_y, p3.normal_z);

        STDesc single_descriptor;
        current_frame_id_++;
        single_descriptor.vertex_A_ = A;
        single_descriptor.vertex_B_ = B;
        single_descriptor.vertex_C_ = C;
        single_descriptor.center_ = (A + B + C) / 3;
        single_descriptor.side_length_ << scale * a, scale * b, scale * c;
        single_descriptor.angle_[0] = fabs(5 * normal_1.dot(normal_2));
        single_descriptor.angle_[1] = fabs(5 * normal_1.dot(normal_3));
        single_descriptor.angle_[2] = fabs(5 * normal_3.dot(normal_2));
        single_descriptor.normal1_ = normal_1;
        single_descriptor.normal2_ = normal_2;
        single_descriptor.normal3_ = normal_3;
        single_descriptor.frame_id_ = current_frame_id_;

        stds_vec.push_back(single_descriptor);

    }
}
   
// Función para verificar y agrupar vértices dentro de un radio
void cluster_vertx(const Eigen::MatrixXf &vertices, std::vector<int> &labels, const float EPSILON) {
    //std::cout<<"epsilon: "<<EPSILON<<std::endl;
    const int num_points = vertices.rows();
    labels.assign(num_points, -1); // Inicializar etiquetas a -1 (no visitado)
    int current_label = 0;

    for (int i = 0; i < num_points; ++i) {
        if (labels[i] != -1) continue; // Si ya está etiquetado, continuar al siguiente

        // Etiquetar el punto actual con una nueva etiqueta de cluster
        labels[i] = current_label;

        // Recorrer todos los puntos desde el siguiente al actual para encontrar vecinos
        for (int j = i + 1; j < num_points; ++j) {
            if (calcularDistancia(vertices.row(i).transpose(), vertices.row(j).transpose()) <= EPSILON) {
                labels[j] = current_label; // Etiquetar el punto vecino con la misma etiqueta de cluster
            }
        }
        current_label++;
    }


}

// Función para promediar los vértices agrupados por labels
Eigen::MatrixXf promediarVertices(const Eigen::MatrixXf &vertices, const std::vector<int> &labels) {
    std::map<int, Eigen::Vector3f> sum_vertices;
    std::map<int, int> count_vertices;

    // Sumarizar los vértices por label
    for (int i = 0; i < vertices.rows(); ++i) {
        int label = labels[i];
        if (label >= 0) {
            if (sum_vertices.find(label) == sum_vertices.end()) {
                sum_vertices[label] = Eigen::Vector3f::Zero();
                count_vertices[label] = 0;
            }
            sum_vertices[label] += vertices.row(i).transpose();
            count_vertices[label] += 1;
        }
    }

    // Crear una nueva matriz de vértices con los promedios
    Eigen::MatrixXf new_vertices(vertices.rows(), vertices.cols());

    for (int i = 0; i < vertices.rows(); ++i) {
        int label = labels[i];
        if (label >= 0 && count_vertices[label] > 0) {
            new_vertices.row(i) = (sum_vertices[label] / count_vertices[label]).transpose();
        } else {
            new_vertices.row(i) = vertices.row(i);
        }
    }

    return new_vertices;
}

void updateMatrixAndKDTreeWithFiltering(Eigen::MatrixXf& mat, std::unique_ptr<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>>& index, std::deque<STDesc>& std_local_map,  ConfigSetting config_setting) {
    int num_desc = std_local_map.size();

    /////////////////// Filtrado de vértices
    Eigen::MatrixXf all_vertices;
    extractVerticesToMatrix(std_local_map, all_vertices);

    // Aplicar cluster a todos los vértices
     std::vector<int> vertex_labels;
    cluster_vertx(all_vertices, vertex_labels, config_setting.epsilon_);


    Eigen::MatrixXf new_vertices = promediarVertices(all_vertices, vertex_labels);

    // Reconstruir std_local_map con los vértices fusionados
    std::deque<STDesc> filtered_std_local_map;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr corner_points(new pcl::PointCloud<pcl::PointXYZINormal>);

    for (size_t i = 0; i < std_local_map.size(); ++i) {
        pcl::PointXYZINormal p1, p2, p3;

        Eigen::Vector3d vertex_A = new_vertices.row(3 * i).cast<double>();
        Eigen::Vector3d vertex_B = new_vertices.row(3 * i + 1).cast<double>();
        Eigen::Vector3d vertex_C = new_vertices.row(3 * i + 2).cast<double>();

        // Supongamos que las normales originales están en std_local_map
        p1.x = vertex_A[0]; p1.y = vertex_A[1]; p1.z = vertex_A[2];
        p1.normal_x = std_local_map[i].normal1_[0]; p1.normal_y = std_local_map[i].normal1_[1]; p1.normal_z = std_local_map[i].normal1_[2];
        p1.intensity = std_local_map[i].vertex_attached_[0];

        p2.x = vertex_B[0]; p2.y = vertex_B[1]; p2.z = vertex_B[2];
        p2.normal_x = std_local_map[i].normal2_[0]; p2.normal_y = std_local_map[i].normal2_[1]; p2.normal_z = std_local_map[i].normal2_[2];
        p2.intensity = std_local_map[i].vertex_attached_[1];

        p3.x = vertex_C[0]; p3.y = vertex_C[1]; p3.z = vertex_C[2];
        p3.normal_x = std_local_map[i].normal3_[0]; p3.normal_y = std_local_map[i].normal3_[1]; p3.normal_z = std_local_map[i].normal3_[2];
        p3.intensity = std_local_map[i].vertex_attached_[2];

        corner_points->points.push_back(p1);
        corner_points->points.push_back(p2);
        corner_points->points.push_back(p3);
    }

    //Usar corner_points para construir stds_vec
    std::vector<STDesc> stds_vec;
    build_std_filter(corner_points, stds_vec, config_setting);
    filtered_std_local_map.assign(stds_vec.begin(), stds_vec.end());
    // Actualizar std_local_map con los descriptores filtrados y fusionados
    std_local_map = std::move(filtered_std_local_map);

    // Actualizar la matriz y el KD-Tree con los descriptores filtrados y fusionados
    num_desc = std_local_map.size();
    mat.resize(num_desc, 24);
    
    for (size_t i = 0; i < std_local_map.size(); ++i) {
        addDescriptorToMatrix(mat, std_local_map[i], i); // aqui se añaden ya los descritproes filtrados al mapa STD_local

    }

    // Recrear el KD-Tree con la matriz actualizada
    index = std::make_unique<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>>(24, std::cref(mat), 10 /* max leaf */);
    index->index_->buildIndex();
}

#endif // _STD_FUNCTIONS_H_

