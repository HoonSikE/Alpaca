package com.example.taxi.ui.payment

import android.util.Log
import androidx.fragment.app.FragmentActivity
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.databinding.FragmentPaymentBinding
import com.example.taxi.di.ApplicationClass
import dagger.hilt.android.AndroidEntryPoint
import kr.co.bootpay.android.Bootpay
import kr.co.bootpay.android.events.BootpayEventListener
import kr.co.bootpay.android.models.BootExtra
import kr.co.bootpay.android.models.BootItem
import kr.co.bootpay.android.models.BootUser
import kr.co.bootpay.android.models.Payload

@AndroidEntryPoint
class PaymentFragment : BaseFragment<FragmentPaymentBinding>(R.layout.fragment_payment) {


    override fun init() {
        totalPayment()
    }

    private fun totalPayment(){
        /** ARS로 결제할 경우 가상결제, Naver Pay로 하면 실제 결제 확인 */
        // BootPay Android용 고정 ID
        var applicationId = "6330fc4dcf9f6d001f9266d5"

        // 일시불
        val extra = BootExtra().setCardQuota("0")

        val items: MutableList<BootItem> = ArrayList()

        var product = "AlpacaTaxi"

//        items.add(BootItem().setName(product).setId("ITEM_CODE_MOUSE").setQty(1).setPrice(ApplicationClass.prefs.fee!!.toDouble()))
        items.add(BootItem().setName(product).setId("ITEM_CODE_MOUSE").setQty(1).setPrice(1000.toDouble()))
        val payload = Payload()

        /** 정보 추가 핵심 부분. OrderName, Product, Price, User, item */
        payload.setApplicationId(applicationId)
            // 주문 이름 설정
            .setOrderName(product)
            // 사용할 PG사, 결제 방식 -> 설정 안 하면 통합 결제
            .setOrderId("1234")
            // Price와 items의 가격정보가 일치해야 한다. (틀리면 바로 오류)
            // 원래는 합계를 입력해야하지만, 어차피 1건이므로 바로 입력
//            .setPrice(ApplicationClass.prefs.fee!!.toDouble())
            .setPrice(1000.toDouble())
            .setUser(getBootUser())
            .setExtra(extra).items = items

        /** 이부분은 Bootpay 관리자 페이지에서만 확인하프로 크게 신경쓰지 않아도 됨. */
        val map: MutableMap<String, Any> = HashMap()
        map["Service Name"] = "알파카"
        map["Product"] = product
//        map["Price"] = ApplicationClass.prefs.fee!!.toDouble().toInt().toString() + "원"
        map["Price"] = "1000원"
        payload.metadata = map

        /** 이부분은 Bootpay 관리자 페이지에서만 확인하프로 크게 신경쓰지 않아도 됨. */
        Bootpay.init(parentFragmentManager, activity?.applicationContext)
            .setPayload(payload)
            .setEventListener(object : BootpayEventListener {
                override fun onCancel(data: String) {
                    Log.d("bootpay", "cancel: $data")
                }
                override fun onError(data: String) {
                    Log.d("bootpay", "error: $data")
                }
                override fun onClose(data: String) {
                    Log.d("bootpay", "close: $data")
                    Bootpay.removePaymentWindow()
                }
                override fun onIssued(data: String) {
                    Log.d("bootpay", "issued: $data")
                }
                override fun onConfirm(data: String): Boolean {
                    Log.d("bootpay", "confirm: $data")
                    return true //재고가 있어서 결제를 진행하려 할때 true (방법 2)
                }
                override fun onDone(data: String) {
                    Log.d("done", data)
                }
            }).requestPayment()
        ApplicationClass.prefs.useCount = ApplicationClass.prefs.useCount?.plus(1)
        findNavController().navigate(R.id.action_paymentFragment_to_userHomeFragment)
    }

    fun getBootUser(): BootUser? {
        val user = BootUser()
        user.id = ApplicationClass.userId
        user.email = ApplicationClass.userId
        user.phone = ApplicationClass.prefs.tel
        user.username = ApplicationClass.prefs.name
        return user
    }
}