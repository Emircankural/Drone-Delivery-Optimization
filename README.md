# Drone Teslimat Optimizasyonu Sistemi
Bu proje, çoklu drone'lar kullanarak teslimat rotalarını optimize eden gelişmiş bir algoritma sistemidir. A* yol bulma algoritması, Constraint Satisfaction Problem (CSP) ve Genetik Algoritma tekniklerini birleştirerek, no-fly zone'lardan kaçınan optimal teslimat rotaları oluşturur.

🚁 Özellikler
Çoklu Drone Desteği: Farklı kapasitelerde birden fazla drone ile eşzamanlı teslimat
No-Fly Zone Kaçınma: A* algoritması ile yasak bölgelerden kaçınan güvenli rota planlama
Akıllı Optimizasyon: Genetik algoritma ile enerji tüketimi ve teslimat süresini minimize etme
Gerçek Zamanlı Kısıtlamalar: Ağırlık limitleri, batarya kapasitesi ve zaman penceresi kontrolü
Görselleştirme: Rotaların ve no-fly zone'ların interaktif harita görünümü
Performans Analizi: Detaylı algoritma performans raporları

📋 Gereksinimler
pip install matplotlib numpy

🚀 Kullanım
# Projeyi klonlayın
git clone https://github.com/emircankural/drone-delivery-optimization.git
cd drone-delivery-optimization

# Gerekli kütüphaneleri yükleyin
pip install matplotlib numpy

# Programı çalıştırın
python drone_optimization.py

# Drone'ları tanımla
drones = [
    Drone(id=0, max_weight=10.0, battery=10000, speed=12.0, start_pos=(50, 50)),
    Drone(id=1, max_weight=8.0, battery=8000, speed=10.0, start_pos=(20, 80))
]

# Teslimat noktalarını tanımla
deliveries = [
    DeliveryPoint(id=0, pos=(30, 40), weight=2.5, priority=5, 
                  time_window=(datetime.time(9, 0), datetime.time(17, 0))),
    DeliveryPoint(id=1, pos=(70, 60), weight=1.8, priority=3,
                  time_window=(datetime.time(10, 0), datetime.time(16, 0)))
]

# No-fly zone'ları tanımla
noflyzones = [
    NoFlyZone(id=0, coordinates=[(40, 40), (60, 40), (60, 60), (40, 60)],
              active_time=(datetime.time(0, 0), datetime.time(23, 59)))
]

# Optimizasyonu çalıştır
best_routes, algorithm_time = genetic_algorithm(drones, deliveries, noflyzones)

# Sonuçları görselleştir
plot_routes(best_routes, noflyzones, deliveries)

📊 Performans Metrikleri
Sistem aşağıdaki metrikleri takip eder:

Teslimat Başarı Oranı: Tamamlanan teslimat yüzdesi
Ortalama Enerji Tüketimi: Drone başına enerji kullanımı
Toplam Mesafe: Tüm rotaların toplam uzunluğu
Algoritma Çalışma Süresi: Optimizasyon süre performansı
🎯 Örnek Senaryolar
Senaryo 1: Küçük Ölçekli Teslimat
5 Drone
20 Teslimat Noktası
2 No-Fly Zone
Senaryo 2: Büyük Ölçekli Teslimat
10 Drone
50 Teslimat Noktası
5 No-Fly Zone
📈 Zaman Karmaşıklığı
A Algoritması*: O(E log V)
Genetik Algoritma: O(G × P × D × A)
Toplam Karmaşıklık: O(G × P × D × T × E log V)
Burada:

G = Nesil sayısı
P = Popülasyon boyutu
D = Drone sayısı
T = Teslimat sayısı
E = Kenar sayısı
V = Düğüm sayısı

🎨 Görselleştirme
Sistem, matplotlib kullanarak aşağıdaki görselleştirmeleri sağlar:

Drone Rotaları: Her drone için farklı renkte rota çizgileri
No-Fly Zone'lar: Kırmızı dolgulu yasak bölgeler
Teslimat Noktaları: Renkli işaretçilerle teslimat konumları
Başlangıç Noktaları: Drone üslerinin konumları
![image](https://github.com/user-attachments/assets/4cbaaa48-7b29-46a2-a27e-60d9578f6ba6)
![image](https://github.com/user-attachments/assets/86509dd1-3364-4d4c-a01e-5948ad8824b7)
![image](https://github.com/user-attachments/assets/39ce3f68-5dcc-410a-9cb0-5bbdc2ed389a)

