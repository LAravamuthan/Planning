(define (problem p01)
(:domain bookWorld)
(:objects
tbot3 - robot
tbot3_init_loc - location
book_8 book_9 book_1 book_2 book_3 book_4 book_5 book_6 book_7 book_18 book_19 book_12 book_13 book_10 book_11 book_16 book_17 book_14 book_15 book_23 book_22 book_21 book_20 book_27 book_26 book_25 book_24 book_28 - book
trolly_10 trolly_9 trolly_8 trolly_7 trolly_6 trolly_5 trolly_4 trolly_3 trolly_2 trolly_1 trolly_14 trolly_13 trolly_12 trolly_11 - bin
book_8_iloc book_9_iloc book_1_iloc book_2_iloc book_3_iloc book_4_iloc book_5_iloc book_6_iloc book_7_iloc book_18_iloc book_19_iloc book_12_iloc book_13_iloc book_10_iloc book_11_iloc book_16_iloc book_17_iloc book_14_iloc book_15_iloc book_23_iloc book_22_iloc book_21_iloc book_20_iloc book_27_iloc book_26_iloc book_25_iloc book_24_iloc book_28_iloc - location
trolly_10_iloc trolly_9_iloc trolly_8_iloc trolly_7_iloc trolly_6_iloc trolly_5_iloc trolly_4_iloc trolly_3_iloc trolly_2_iloc trolly_1_iloc trolly_14_iloc trolly_13_iloc trolly_12_iloc trolly_11_iloc - location
Information_Security Computer_Graphics Computer_Networks_I The_Software_Process Cloud_Computing Artificial_Intelligence Operating_Systems_Architecture - subject
small large - size
)
(:init
(Book_At book_8 book_8_iloc)
(Book_At book_9 book_9_iloc)
(Book_At book_1 book_1_iloc)
(Book_At book_2 book_2_iloc)
(Book_At book_3 book_3_iloc)
(Book_At book_4 book_4_iloc)
(Book_At book_5 book_5_iloc)
(Book_At book_6 book_6_iloc)
(Book_At book_7 book_7_iloc)
(Book_At book_18 book_18_iloc)
(Book_At book_19 book_19_iloc)
(Book_At book_12 book_12_iloc)
(Book_At book_13 book_13_iloc)
(Book_At book_10 book_10_iloc)
(Book_At book_11 book_11_iloc)
(Book_At book_16 book_16_iloc)
(Book_At book_17 book_17_iloc)
(Book_At book_14 book_14_iloc)
(Book_At book_15 book_15_iloc)
(Book_At book_23 book_23_iloc)
(Book_At book_22 book_22_iloc)
(Book_At book_21 book_21_iloc)
(Book_At book_20 book_20_iloc)
(Book_At book_27 book_27_iloc)
(Book_At book_26 book_26_iloc)
(Book_At book_25 book_25_iloc)
(Book_At book_24 book_24_iloc)
(Book_At book_28 book_28_iloc)
(Bin_At trolly_10 trolly_10_iloc)
(Bin_At trolly_9 trolly_9_iloc)
(Bin_At trolly_8 trolly_8_iloc)
(Bin_At trolly_7 trolly_7_iloc)
(Bin_At trolly_6 trolly_6_iloc)
(Bin_At trolly_5 trolly_5_iloc)
(Bin_At trolly_4 trolly_4_iloc)
(Bin_At trolly_3 trolly_3_iloc)
(Bin_At trolly_2 trolly_2_iloc)
(Bin_At trolly_1 trolly_1_iloc)
(Bin_At trolly_14 trolly_14_iloc)
(Bin_At trolly_13 trolly_13_iloc)
(Bin_At trolly_12 trolly_12_iloc)
(Bin_At trolly_11 trolly_11_iloc)
(Book_Subject book_8 Artificial_Intelligence)
(Book_Size book_8 large)
(Book_Subject book_9 Information_Security)
(Book_Size book_9 small)
(Book_Subject book_1 Operating_Systems_Architecture)
(Book_Size book_1 small)
(Book_Subject book_2 Operating_Systems_Architecture)
(Book_Size book_2 small)
(Book_Subject book_3 Operating_Systems_Architecture)
(Book_Size book_3 large)
(Book_Subject book_4 Operating_Systems_Architecture)
(Book_Size book_4 large)
(Book_Subject book_5 Artificial_Intelligence)
(Book_Size book_5 small)
(Book_Subject book_6 Artificial_Intelligence)
(Book_Size book_6 small)
(Book_Subject book_7 Artificial_Intelligence)
(Book_Size book_7 large)
(Book_Subject book_18 Computer_Graphics)
(Book_Size book_18 small)
(Book_Subject book_19 Computer_Graphics)
(Book_Size book_19 large)
(Book_Subject book_12 Information_Security)
(Book_Size book_12 large)
(Book_Subject book_13 Computer_Networks_I)
(Book_Size book_13 small)
(Book_Subject book_10 Information_Security)
(Book_Size book_10 small)
(Book_Subject book_11 Information_Security)
(Book_Size book_11 large)
(Book_Subject book_16 Computer_Networks_I)
(Book_Size book_16 large)
(Book_Subject book_17 Computer_Graphics)
(Book_Size book_17 small)
(Book_Subject book_14 Computer_Networks_I)
(Book_Size book_14 small)
(Book_Subject book_15 Computer_Networks_I)
(Book_Size book_15 large)
(Book_Subject book_23 Cloud_Computing)
(Book_Size book_23 large)
(Book_Subject book_22 Cloud_Computing)
(Book_Size book_22 small)
(Book_Subject book_21 Cloud_Computing)
(Book_Size book_21 small)
(Book_Subject book_20 Computer_Graphics)
(Book_Size book_20 large)
(Book_Subject book_27 The_Software_Process)
(Book_Size book_27 large)
(Book_Subject book_26 The_Software_Process)
(Book_Size book_26 small)
(Book_Subject book_25 The_Software_Process)
(Book_Size book_25 small)
(Book_Subject book_24 Cloud_Computing)
(Book_Size book_24 large)
(Book_Subject book_28 The_Software_Process)
(Book_Size book_28 large)
(Bin_Subject trolly_10 Computer_Graphics)
(Bin_Size trolly_10 small)
(Bin_Subject trolly_9 Computer_Graphics)
(Bin_Size trolly_9 large)
(Bin_Subject trolly_8 Computer_Networks_I)
(Bin_Size trolly_8 small)
(Bin_Subject trolly_7 Computer_Networks_I)
(Bin_Size trolly_7 large)
(Bin_Subject trolly_6 Information_Security)
(Bin_Size trolly_6 small)
(Bin_Subject trolly_5 Information_Security)
(Bin_Size trolly_5 large)
(Bin_Subject trolly_4 Artificial_Intelligence)
(Bin_Size trolly_4 small)
(Bin_Subject trolly_3 Artificial_Intelligence)
(Bin_Size trolly_3 large)
(Bin_Subject trolly_2 Operating_Systems_Architecture)
(Bin_Size trolly_2 small)
(Bin_Subject trolly_1 Operating_Systems_Architecture)
(Bin_Size trolly_1 large)
(Bin_Subject trolly_14 The_Software_Process)
(Bin_Size trolly_14 small)
(Bin_Subject trolly_13 The_Software_Process)
(Bin_Size trolly_13 large)
(Bin_Subject trolly_12 Cloud_Computing)
(Bin_Size trolly_12 small)
(Bin_Subject trolly_11 Cloud_Computing)
(Bin_Size trolly_11 large)
(Robot_At tbot3 tbot3_init_loc)
(Empty_Basket tbot3)
)
(:goal ENTER YOUR GOAL FORMULA HERE )
)