/*
Initial values of variables:
    status = -1 // Uçuşa Hazır değil
    start_button = 0    // bağlı olduğu butonun değişimine göre 1 olacak.
    ayrilma_status = 0    // bağlı olduğu butonun değişimine göre 1 olacak.
    buzzer_cal = 0
    ayrilma_basla = 0
    manual_ayrilma = 0  // Dışarıdan gelen bir konut ile 1'e dönüşür
    hata_kodu_hiz_biti = 0

    // tolerans variable'ları sonradan atanacak
    //yükseklik1 ve inis_hizi verisi telemetri paketindeki veri
*/




if(((inis_hizi > tolerans3) and (uydu_statusu > 1) and (ayrilma_status == 0) and (400 + ayrilma_toleransi)) or (mauel_ayrilma == 1))   //Uydu Status 3: Ayrılma
{
    uydu_statusu = 3
    ayrilma_basla = 1
    hata_kodu_hiz_biti = ((14 <= inis_hizi) or (inis_hizi < 12)) ? 1 : 0   // inis_hizi 14'ten büyük ya da 12 den küçükse 1, diğer türlü 0'a eşit
}

else if((abs(inis_hizi) < tolerans0) and (uydu_statusu =< 0) and (start_button == 1) and (yukseklik1 < 5)) // uydu_Statusu 0: Uçuşa Hazır
{
    uydu_statusu = 0
}

else if((inis_hizi < ((-1)*tolerans1)) and (2 > uydu_statusu >= 0) and (yukseklik1 >= 5)) // uydu_Statusu 1: Yükselme
{
    uydu_statusu = 1
}

else if((inis_hizi > toleran2) and (uydu_statusu > 0) and (ayrilma_status == 0))    // uydu_Statusu 2: MUY iniş
{
    uydu_statusu = 2
    hata_kodu_hiz_biti = ((14 <= inis_hizi) or (inis_hizi < 12)) ? 1 : 0   // inis_hizi 14'ten büyük ya da 12 den küçükse 1, diğer türlü 0'a eşit
}

else if((inis_hizi > tolerans4) and (uydu_statusu > 2) and (ayrilma_status == 1))    // uydu_Statusu 4: Görev Yükü iniş
{
    uydu_statusu = 4
    ayrilma_basla = 0
    hata_kodu_hiz_biti = ((8 < inis_hizi) or (inis_hizi < 6)) ? 1 : 0   // inis_hizi 8'ten büyük ya da 6 den küçükse 1, diğer türlü 0'a eşit
}

else if ((abs(inis_hizi) < tolerans5) and (status >= 3))    //Uydu_status 5: Kurtarma
{
    uydu_statusu = 5
    buzzer_cal = 1 // Ya'da byzzer pini'ni HIGH'a çevir
}