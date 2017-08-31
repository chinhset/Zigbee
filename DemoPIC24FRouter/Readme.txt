//Writen by dat_a3cbq91
//18/9/2012
//instruction for project

1. Tinh nang.
Node nay duoc setup de lam Router binh thuong hoac Router_EMB
1.1. Router_EMB.
Cac tinh nang bao gom:
- Gia nhap mang
- Lam nhiem vu trung gian day cac ban tin thong bao gia nhap mang, trang thai cac
    node mang len bo nhung.
- Cho phep cac thiet bi khac gia nhap mang Zigbee.
- Lam nhiem vu trung gian gui lenh tu bo nhung toi cac node mang.

1.2. Router binh thuong.
Node co tinh nang sau:
- Tinh nang thu thap nhiet do-do am: can dinh nghia USE_SHT10 trong file zigbee.def
  Mot cach dinh ki, khi timer1 tran 25 lan, MCU se tien hanh doc du lieu tu cam
  bien SHT10 thong qua giao tiep I2C. Neu lan doc du lieu sau ma co su chenh lech
  1 do C so voi lan doc truoc thi se tien hanh gui du lieu ve Router-emboard
- Tinh nang phat hien khoi: can dinh nghia USE_MQ6 trong file zigbee.def
  Mot cach dinh ki, khi timer1 tran 25 lan se kich hoat bo ADC lay mau tin hieu tu
  MQ6. Khi da lay mau xong, se kich hoat cho gia tri dem so lan tran timer1 len 24
  nhu vay, gan nhu ngay sau do, timer1 tiep tuc tran, luc nay, ADC se lay mau dien ap nguon.
- Tinh nang do dien ap nguon: Can dinh nghia ENERGY_TRACKING. Sau khi lay mau tin hieu
  tu cam bien khoi, ADC se lay mau dien ap nguon.
- Tinh nang bat tat bom tuoi: can dinh nghia USE_CONTROL_PUMP. Khi nhan duoc lenh,
  neu duoc dinh nghia USE_CONTROL_PUMP, no se tien hanh gui cac ki tu A,B,C,S xuong
  mach dieu khien dong co thong qua UART. Sau do, no se tien hanh gui ban tin thong
  bao trang thai ve router-emboard. Nhung neu khong dinh nghia USE_CONTROL_PUMP thi
  no se tien hanh gui thong bao tro lai router-emboard la no khong ho tro viec bat/tat
  van tuoi.

2. Lich su phat trien.
2.1. Ngay 18/9/2012.
- Cong viec: Phat trien code Router_EMB.
- Ket qua:
    + Ket qua test cho thay Route_EMB gia nhap mang on dinh vao Zigbee Coordinator.
    + Cho phep cac thiet bi khac gia nhap vao mang. Cac thiet bi gia nhap deu thong bao ve ZR_EMB
    + ZR_EMB da co the nhan cac ban tin trang thai, nhiet do-do am.
- Han che:
    + Chua gui duoc tin nhan toi cac node khac, ma chi nhan duoc ban tin tu cac
        node khac gui toi.

2.2. Ngay 20/11/2012.
   Ma nguon da duoc chinh sua voi cac tinh nang sau:
    - gui ban tin dinh ki (chua cac du lieu nhiet do, do am, dien ap) ve may tinh nhung voi
      voi chu ki 20s. Chu ki nay co the thay doi bang viec thay doi gia tri trong file zigbee.def
    - gui ban tin theo yeu cau (bao gom du lieu nhiet do, do am, dien ap) ve may tinh nhung.
    - gui ban tin canh bao cac su kien (bao gom canh bao co chay, canh bao het nang luong)
    cac tinh nang tren duoc dinh thoi bang timer3:timer2 tao thanh bo dem thoi gian 32 bit,
    cho phep tao ngat voi chu ki toi da len toi hon 72 tieng.
2.3. Ngay 27/11/2012.
    Ma nguon da duoc test can than cac tinh nang sau:
    - gui ban tin dinh ki ve bo nhung.
    - nhan duoc lenh yeu cau lay du lieu nhiet do - do am tu bo nhung.
    - lay du lieu tu cam bien va gui tra cho bo nhung.
    - nhan duoc lenh bat/tat bom tu bo nhung.
    - gui thong bao trang thai bat/tat bom.
    - gui thong bao ve trang thai dien ap.
