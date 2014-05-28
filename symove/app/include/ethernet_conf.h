#define ETHERNET_DEFAULT_TILE stdcore[16]
#define PORT_ETH_RXCLK on stdcore[16]: XS1_PORT_1J
#define PORT_ETH_RXD on stdcore[16]: XS1_PORT_4E
#define PORT_ETH_TXD on stdcore[16]: XS1_PORT_4F
#define PORT_ETH_RXDV on stdcore[16]: XS1_PORT_1K
#define PORT_ETH_TXEN on stdcore[16]: XS1_PORT_1L
#define PORT_ETH_TXCLK on stdcore[16]: XS1_PORT_1I
#define PORT_ETH_MDIO on stdcore[16]: XS1_PORT_4D //New ethernet
//#define PORT_ETH_MDIO on stdcore[16]: XS1_PORT_1M // Old ethernet
#define PORT_ETH_MDC on stdcore[16]: XS1_PORT_1N
#define PORT_ETH_INT on stdcore[16]: XS1_PORT_1O
#define PORT_ETH_ERR on stdcore[16]: XS1_PORT_1P



#define ETHERNET_DEFAULT_PHY_ADDRESS 0

#if !defined(PORT_ETH_RST_N) && defined(PORT_ETH_RSTN)
#define PORT_ETH_RST_N PORT_ETH_RSTN
#endif

#ifndef ETHERNET_DEFAULT_CLKBLK_0
#define ETHERNET_DEFAULT_CLKBLK_0 on ETHERNET_DEFAULT_TILE: XS1_CLKBLK_1
#endif

#ifndef ETHERNET_DEFAULT_CLKBLK_1
#define ETHERNET_DEFAULT_CLKBLK_1 on ETHERNET_DEFAULT_TILE: XS1_CLKBLK_2
#endif

#if !defined(PORT_ETH_MDIO) && defined(PORT_ETH_RST_N_MDIO)
#define PORT_ETH_MDIO PORT_ETH_RST_N_MDIO
#endif

#if !defined(PORT_ETH_ERR) && defined(PORT_ETH_RXER)
#define PORT_ETH_ERR PORT_ETH_RXER
#endif

#ifndef PORT_ETH_FAKE
#define PORT_ETH_FAKE on ETHERNET_DEFAULT_TILE: XS1_PORT_8C
#endif


// Ethernet Ports
#define ETHERNET_DEFAULT_MII_INIT_full { \
  ETHERNET_DEFAULT_CLKBLK_0, \
  ETHERNET_DEFAULT_CLKBLK_1, \
\
    PORT_ETH_RXCLK,                             \
    PORT_ETH_ERR,                               \
    PORT_ETH_RXD,                               \
    PORT_ETH_RXDV,                              \
    PORT_ETH_TXCLK,                             \
    PORT_ETH_TXEN,                              \
    PORT_ETH_TXD \
}

#define ETHERNET_DEFAULT_MII_INIT_lite { \
  ETHERNET_DEFAULT_CLKBLK_0, \
  ETHERNET_DEFAULT_CLKBLK_1, \
\
    PORT_ETH_RXCLK,                             \
    PORT_ETH_ERR,                               \
    PORT_ETH_RXD,                               \
    PORT_ETH_RXDV,                              \
    PORT_ETH_TXCLK,                             \
    PORT_ETH_TXEN,                              \
    PORT_ETH_TXD,                               \
    PORT_ETH_FAKE \
}


#define ETHERNET_DEFAULT_MII_INIT ADD_SUFFIX(ETHERNET_DEFAULT_MII_INIT,ETHERNET_DEFAULT_IMPLEMENTATION)


#if SMI_COMBINE_MDC_MDIO
#define ETHERNET_DEFAULT_SMI_INIT {ETHERNET_DEFAULT_PHY_ADDRESS, \
                                   PORT_ETH_MDIOC}
#else
#define ETHERNET_DEFAULT_SMI_INIT {ETHERNET_DEFAULT_PHY_ADDRESS, \
                                   PORT_ETH_MDIO,       \
                                   PORT_ETH_MDC}
#endif
