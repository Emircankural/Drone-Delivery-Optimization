Drone Teslimat Optimizasyonu
Bu proje, birden fazla drone ile teslimat yapılırken uçuş yasağı bölgelerinden kaçınarak teslimat sürelerini ve enerji tüketimini minimize eden, aynı zamanda drone kapasite ve zaman kısıtlamalarını dikkate alan bir rota optimizasyonu sistemini geliştirmeyi amaçlamaktadır.

İçindekiler:
Proje Amacı

Kullanılan Teknolojiler

Başlangıç

Kullanım

Algoritmalar ve Modeller

Katkıda Bulunma

Lisans

Proje Amacı
Bu proje, drone teslimat sistemleri için optimizasyon çözümleri sunmak amacıyla geliştirilmiştir. Amaç, drone'ların çeşitli kısıtlamalarla (batarya kapasitesi, taşıma kapasitesi, teslimat zaman pencereleri ve no-fly zone'lar) güvenli ve verimli rotalar çizmesidir. Aşağıdaki faktörler göz önünde bulundurulmuştur:

Drone Kapasitesi ve Batarya: Her drone'un taşıma kapasitesi ve batarya sınırları.

No-Fly Zone: Uçuş yasağı bölgelerinden kaçınılması gerekmektedir.

Teslimat Zaman Penceresi: Teslimatlar belirli zaman dilimlerinde yapılmalıdır.

Enerji Tüketimi: Rotalar enerji verimliliğine göre optimize edilmiştir.

Proje, A* algoritması ve genetik algoritma kullanılarak rotaların optimize edilmesini sağlar.

Kullanılan Teknolojiler
Bu proje aşağıdaki teknolojiler ve kütüphaneleri kullanmaktadır:

Python 3.x: Programlama dili

matplotlib: Görselleştirme ve grafikler için

numpy: Matematiksel işlemler ve veri işleme için

heapq: A* algoritması için öncelikli kuyruk (priority queue) yönetimi

datetime: Teslimat zaman pencereleri ve uçuş yasağı sürelerinin yönetimi

Başlangıç
Projeyi çalıştırmak için aşağıdaki adımları takip edebilirsiniz:

1. Gereksinimler
Projenin çalışabilmesi için öncelikle Python 3.x'in sisteminizde yüklü olduğundan emin olun. Gerekli Python kütüphanelerini yüklemek için aşağıdaki komutu kullanabilirsiniz:

bash
Kopyala
pip install -r requirements.txt
2. Verilerin Hazırlanması
Projede rastgele drone, teslimat noktası ve no-fly zone verileri üretilmiştir. generate_random_drones, generate_random_deliveries ve generate_random_noflyzones fonksiyonları, veri üretimini sağlar. Eğer kendi verilerinizi kullanmak isterseniz, bu fonksiyonları düzenleyebilirsiniz.

3. Projeyi Çalıştırma
Projeyi çalıştırmak için ana fonksiyonu çalıştırabilirsiniz. main() fonksiyonu, senaryoları başlatır, rotaları optimize eder, performansı analiz eder ve sonuçları görselleştirir.

bash
Kopyala
python drone_optimization.py
4. Çıktılar
Optimizasyon sonucunda drone’ların rotaları ve teslimat bilgileri görselleştirilir.

Performans analizi yapılır (tamamlanan teslimat oranı, enerji tüketimi, mesafe vb.).

Kullanım
Genetik Algoritma: Drone’lara teslimatları rastgele dağıtmak için genetik algoritma kullanılır. Teslimat noktaları, drone’ların kapasite ve batarya sınırlarına göre sıralanır.

A Algoritması:* Her drone’un teslimat noktası arasında güvenli bir yol oluşturmak için A* algoritması kullanılır. A* algoritması, no-fly zone'ları ve teslimat önceliklerini dikkate alır.

Performans Analizi: Tamamlanan teslimat sayısı, enerji tüketimi, mesafe ve algoritma çalışma süresi gibi metrikler raporlanır.

Algoritmalar ve Modeller
A* Algoritması:
A* algoritması, başlangıç ve hedef noktası arasındaki en kısa rotayı bulmak için kullanılan bir yol planlama algoritmasıdır. Bu projede, uçuş yasağı bölgelerinden kaçınmak için ek bir ceza uygulanarak, güvenli ve optimal rotalar oluşturulmuştur.

Genetik Algoritma:
Teslimatları en verimli şekilde dağıtmak için genetik algoritmalar kullanılır. Bu algoritma, başlangıç popülasyonu oluşturur, ebeveynler arasında çaprazlama ve mutasyon işlemleri yapar, ve uygunluk fonksiyonlarına göre en iyi çözümü bulur.

No-Fly Zone:
No-fly zone'lar, çokgen olarak tanımlanır ve her drone, teslimat rotası boyunca bu bölgelerden kaçınmak zorundadır. A* algoritması, yolun bu bölgelerle kesişip kesişmediğini kontrol eder.
![image](https://github.com/user-attachments/assets/4cbaaa48-7b29-46a2-a27e-60d9578f6ba6)
![image](https://github.com/user-attachments/assets/86509dd1-3364-4d4c-a01e-5948ad8824b7)
![image](https://github.com/user-attachments/assets/39ce3f68-5dcc-410a-9cb0-5bbdc2ed389a)

