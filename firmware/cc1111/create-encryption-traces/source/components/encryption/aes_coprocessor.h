// C language code:
// Data structure for DMA descriptor
/*
typedef struct {
unsigned char SRCADDRH;
unsigned char SRCADDRL;
unsigned char DESTADDRH;
unsigned char DESTADDRL;
unsigned char VLEN : 3;
unsigned char LENH : 5;
unsigned char LENL : 8;
unsigned char WORDSIZE : 1;
unsigned char TMODE : 2;
unsigned char TRIG : 5;
unsigned char SRCINC : 2;
unsigned char DESTINC : 2;
unsigned char IRQMASK : 1;
unsigned char M8 : 1;
unsigned char PRIORITY : 2;
} DMA_DESC;
// Allocate DMA descriptors for AES download and upload
DMA_DESC __xdata dma_ch_0, __xdata dma_ch_1;

// Link each allocated DMA channel descriptor with its corresponding
// DMA configuration register.

DMA0CFGH = (unsigned char)((unsigned short)&dma_ch_0 >> 8);
DMA0CFGL = (unsigned char)((unsigned short)&dma_ch_0 & 0x00FF);
DMA1CFGH = (unsigned char)((unsigned short)&dma_ch_1 >> 8);
DMA1CFGL = (unsigned char)((unsigned short)&dma_ch_1 & 0x00FF);

// Setup DMA channel 0 to download data to the AES coprocessor
dma_ch_0.DESTADDRH = 0xDF; // High byte address of ENCDI (AES data input)
dma_ch_0.DESTADDRL = 0xB1; // Low byte Address of ENCDI (AES data input)
dma_ch_0.VLEN = 0x00;      // Use LEN for transfer count
dma_ch_0.LENH = 0;         // Set to 0, because this app report assumes
                           // size < 256. LENL is set by the code shown
                           // in Example 2 and Example 3.
dma_ch_0.WORDSIZE = 0x00;  // Perform byte-wise transfer
dma_ch_0.TMODE = 0x00;     // Transfer a single byte after each
                           // DMA trigger.
dma_ch_0.TRIG = 29;        // AES coprocessor requests download input data
dma_ch_0.SRCINC = 0x01;    // Increment source pointer by 1 byte after
                           // each transfer.
dma_ch_0.DESTINC = 0x00;   // Do not increment destination pointer:
                           // points to AES ENCDI register.
dma_ch_0.IRQMASK = 0x00;   // Disable DMA interrupt to the CPU
dma_ch_0.M8 = 0x00;        // Use all 8 bits for transfer count
dma_ch_0.PRIORITY = 0x01;  // Guaranteed, DMA at least every second try
                           // Setup DMA channel 1 to upload data from the AES coprocessor
dma_ch_1.SRCADDRH = 0xDF;  // High byte address of ENCDO (AES data output)
dma_ch_1.SRCADDRL = 0xB2;  // Low byte Address of ENCDO (AES data output)
dma_ch_1.VLEN = 0x00;      // Use LEN for transfer count
dma_ch_1.LENH = 0;         // Set to 0, because this app report assumes
                           // size < 256. LENL is set by the code shown
                           // in Example 2 and Example 3.
dma_ch_1.WORDSIZE = 0x00;  // Perform byte-wise transfer
dma_ch_1.TMODE = 0x00;     // Transfer a single byte after each
                           // DMA trigger.
dma_ch_1.TRIG = 30;        // AES coprocessor requests upload output data.
dma_ch_1.SRCINC = 0x00;    // Do not increment source pointer:
                           // points to AES ENCDO.
dma_ch_1.DESTINC = 0x01;   // Increment destination pointer by 1 byte
                           // after each transfer.
dma_ch_1.IRQMASK = 0x00;   // Disable DMA interrupt to the CPU
dma_ch_1.M8 = 0x00;        // Use all 8 bits for transfer count
dma_ch_1.PRIORITY = 0x01;  // Guaranteed, DMA at least every second try
*/