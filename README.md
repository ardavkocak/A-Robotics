# A-Robotics
A Star Algorithm
Robot Yol Planlama Projesi Raporu
1. Giriş
Bu rapor, 10 robot için bir yol planlama ve navigasyon sistemi uygulamasını detaylandırmaktadır. Amaç, tüm robotların kendi hedeflerine 10 saniye içinde ulaşmasını sağlamak, çarpışmalardan kaçınmak ve fiziksel kısıtlamalarına uygun şekilde hareket etmelerini sağlamaktır.
2. Problem Tanımı
Bu proje aşağıdaki zorlukları kapsamaktadır:
•	Rastgele yerleştirilmiş engellerle bir ızgara harita oluşturulması.
•	Her robot için rastgele başlangıç ve hedef pozisyonlarının atanması.
•	A* algoritması ile çarpışmasız yolların hesaplanması.
•	Robotların diferansiyel sürüş kinematikleri kullanılarak yollarını izlemelerinin simüle edilmesi.
•	Tüm robotların mesafelerine göre hızlarını ayarlayarak 10 saniye içinde hedeflerine ulaşmalarını sağlamak.
3. Yöntem
3.1 Harita Oluşturma
•	Izgara Boyutu: 100x100.
•	Engeller: Rastgele yerleştirilmiş 200 engel.
•	Görselleştirme: Engeller siyah, serbest alanlar beyaz olarak gösterilmiştir.
3.2 Yol Planlama
•	Algoritma: A* algoritması çarpışmasız yolların bulunmasında kullanılmıştır.
•	Heuristik Fonksiyon: Manhattan mesafesi.
•	Yol Saklama: Her robotun yolu, ızgara koordinatları olarak saklanmıştır.
3.3 Diferansiyel Sürüş Simülasyonu
•	Kinematik Model: 
o	Doğrusal hız (v) ve açısal hız (ω) hesaplanmıştır.
o	Tekerlek hızları (v_left, v_right) şu formüllerle türetilmiştir:
vleft=v−ω⋅L2,vright=v+ω⋅L2v_{\text{left}} = v - \frac{\omega \cdot L}{2}, \quad v_{\text{right}} = v + \frac{\omega \cdot L}{2} 
Burada LL, tekerlekler arası mesafedir.
•	Zaman Kısıtlaması: Her robot, hedefe tam olarak 10 saniyede ulaşacak şekilde hızını dinamik olarak ayarlamaktadır.
3.4 Çarpışma Tespiti
•	Yöntem: 
o	Robot merkezleri arasındaki Öklid mesafesi hesaplanmıştır.
o	Mesafe, robot yarıçaplarının toplamından küçükse çarpışma olduğu kabul edilmiştir.
•	Görselleştirme: Çarpışma noktaları ızgara üzerinde işaretlenmiştir.
4. Sonuçlar
4.1 Görselleştirme
•	Harita: Başlangıç ve hedef pozisyonlarını içeren başlangıç haritası.
•	Yollar: Her robotun yolu farklı bir renk ile gösterilmiştir.
4.2 Performans Ölçütleri
•	Çarpışma Tespiti: Tespit edilen çarpışmalar robot kimlikleri ve çarpışma koordinatları ile kaydedilmiştir.
•	Hız Profilleri: 
o	Her robot için doğrusal ve açısal hız profilleri çizilmiştir.
o	Sol ve sağ tekerlek hız profilleri ayrı ayrı gösterilmiştir.
4.3 Gözlemler
•	Tüm robotlar, 10 saniyelik süre sınırı içinde başarıyla hedeflerine ulaşmıştır.
•	Farklı yol uzunluklarına sahip robotlar, hızlarını dinamik olarak ayarlayarak senkronizasyon sağlamıştır.
5. Sonuç
Bu proje, bir ızgara tabanlı ortamda çok robotlu yol planlama ve navigasyon için etkili bir yaklaşımı göstermektedir. Öne çıkan noktalar şunlardır:
•	A* algoritmasının etkin entegrasyonu.
•	Diferansiyel sürüş kinematiklerinin gerçekçi robot hareketleri için kullanılması.
•	Zaman kısıtlamalarını karşılamak için dinamik hız ölçeklendirme.
6. Gelecek Çalışmalar
•	Gelişmiş çarpışma önleme stratejilerinin uygulanması.
•	Dinamik engellerin dahil edilmesi.
•	Gerçek zamanlı animasyonlarla görselleştirmenin geliştirilmesi.
7. Ekler
7.1 Temel Denklemler
•	Doğrusal ve Açısal Hızlar: v=mesafezaman,ω=atan2(dy,dx)v = \frac{\text{mesafe}}{\text{zaman}}, \quad \omega = \text{atan2}(dy, dx) 
•	Tekerlek Hızları: vleft=v−ω⋅L2,vright=v+ω⋅L2v_{\text{left}} = v - \frac{\omega \cdot L}{2}, \quad v_{\text{right}} = v + \frac{\omega \cdot L}{2} 
7.2 MATLAB Kodu
Tam MATLAB kodu ekli dosyada verilmiştir.
7.3 Şekiller
•	Başlangıç ve hedef pozisyonlarını içeren harita.
•	Yol trajektoryaları.
•	Her robotun hız profilleri.
•	Çarpışma görselleştirmesi.

